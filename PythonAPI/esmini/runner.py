# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Main co-simulation orchestrator bridging esminiLib and CARLA."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import shutil
import time
from typing import Any, Callable

from .actor_manager import ActorManager
from .coordinates import sync_opendrive_map
from .environment_sync import EnvironmentSynchronizer
from .esmini_lib import (
    EsminiLib,
    SEStoryBoardElementState,
    SEStoryBoardElementType,
)

try:
    import carla as _carla_module
except ImportError:
    _carla_module = None


@dataclass
class RunnerConfig:
    """Configuration for ``CarlaEsminiRunner``."""

    xosc_path: str | Path
    host: str = "127.0.0.1"
    port: int = 2000
    client_timeout: float = 15.0
    delta_seconds: float = 0.05
    sync_mode: bool = True
    passive_tick: bool = False
    reload_world: bool = False
    time_scale: float = 1.0
    max_sim_time: float | None = None

    # Ego vehicle options
    ego_name: str | None = None
    ego_role_name: str = "hero"
    ego_blueprint: str | None = None
    spawn_ego: bool = True
    wait_for_ego: bool = False
    ego_wait_timeout: float = 30.0
    teleport_external_ego: bool = True
    keep_ego_on_exit: bool = False
    enable_ros2: bool = False
    autoware_mode: bool = False
    autoware_sensors: bool = False

    # Visualization & alignment options
    follow_ego: bool = False
    draw_ghost: bool = False
    apply_center_offset: bool = True
    esmini_viewer: int = 0
    disable_esmini_log: bool = False
    record_dat_path: str | None = None
    random_seed: int | None = None

    # Scenario parameters & search paths
    parameters: dict[str, Any] = field(default_factory=dict)
    additional_paths: list[str | Path] = field(default_factory=list)
    esmini_lib_path: str | Path | None = None
    dry_run: bool = False


class CarlaEsminiRunner:
    """Co-simulates an ASAM OpenSCENARIO 1.x (``.xosc``) file using ``esminiLib`` and CARLA."""

    def __init__(
        self,
        config: RunnerConfig,
        carla_client: Any = None,
        carla_world: Any = None,
        esmini: EsminiLib | None = None,
        carla_mod: Any = None,
    ) -> None:
        self.config = config
        self.carla_mod = carla_mod if carla_mod is not None else _carla_module
        self.client = carla_client
        self.world = carla_world
        self.esmini = esmini or EsminiLib(lib_path=config.esmini_lib_path)

        self.actor_manager: ActorManager | None = None
        self.env_sync: EnvironmentSynchronizer | None = None

        self._original_world_settings: Any = None
        self._temp_xodr_dir: Path | None = None
        self._is_setup: bool = False
        self.triggered_conditions: list[tuple[str, float]] = []
        self.storyboard_transitions: list[
            tuple[str, SEStoryBoardElementType, SEStoryBoardElementState, str]
        ] = []
        self.step_callbacks: list[Callable[[float], None]] = []

    # ------------------------------------------------------------------
    # Setup & Initialization
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Connect to CARLA, sync OpenDRIVE map, initialize esminiLib, and spawn actors."""
        xosc_file = Path(self.config.xosc_path).resolve()
        if not xosc_file.is_file():
            raise FileNotFoundError(f"OpenSCENARIO file not found: {xosc_file}")

        # 1. Connect to CARLA server (unless running in offline dry_run mode or pre-supplied)
        if not self.config.dry_run and self.world is None:
            if self.client is None:
                if self.carla_mod is None:
                    raise RuntimeError(
                        "The 'carla' Python package is not installed and no carla_client was provided. "
                        "Use dry_run=True for offline esmini execution or install the carla wheel."
                    )
                self.client = self.carla_mod.Client(self.config.host, self.config.port)
                self.client.set_timeout(self.config.client_timeout)
            self.world = self.client.get_world()

        # 2. Configure esmini options & search paths
        self.esmini.clear_paths()
        if self.config.disable_esmini_log:
            self.esmini.set_log_file_path("")
            self.esmini.log_to_console(False)
        if self.config.record_dat_path:
            self.esmini.set_dat_file_path(self.config.record_dat_path)
        if self.config.random_seed is not None:
            self.esmini.set_seed(self.config.random_seed)

        # 3. Synchronize OpenDRIVE map between CARLA and esmini
        self.world, self._temp_xodr_dir = sync_opendrive_map(
            xosc_path=xosc_file,
            esmini=self.esmini,
            carla_client=self.client,
            carla_world=self.world,
            reload_carla_world=self.config.reload_world,
            additional_search_paths=self.config.additional_paths,
        )

        # 4. Configure CARLA synchronous mode
        if self.world is not None and not self.config.passive_tick:
            self._original_world_settings = self.world.get_settings()
            if self.config.sync_mode:
                new_settings = self.world.get_settings()
                new_settings.synchronous_mode = True
                new_settings.fixed_delta_seconds = float(self.config.delta_seconds)
                self.world.apply_settings(new_settings)

        # 5. Register parameter override, condition, and storyboard callbacks in esmini
        if self.config.parameters:
            self.esmini.register_parameter_declaration_callback(
                self._apply_parameter_overrides
            )
        else:
            self.esmini.register_parameter_declaration_callback(None)

        self.esmini.register_condition_callback(self._on_condition_triggered)
        self.esmini.register_storyboard_element_state_change_callback(
            self._on_storyboard_state_change
        )

        # 6. Initialize EnvironmentSynchronizer (populated as soon as SE_Init resolves XML)
        self.env_sync = EnvironmentSynchronizer(
            esmini=self.esmini,
            carla_world=self.world,
            xosc_path=xosc_file,
            carla_mod=self.carla_mod,
        )

        # 7. Initialize esmini scenario engine
        init_res = self.esmini.init(
            osc_filename=xosc_file,
            disable_ctrls=False,
            use_viewer=self.config.esmini_viewer,
            threads=0,
            record=bool(self.config.record_dat_path),
        )
        if init_res != 0:
            raise RuntimeError(
                f"esmini SE_Init failed (return code {init_res}) for scenario: {xosc_file}"
            )

        # 8. Index XML for EnvironmentAction / TrafficSignalAction and apply Init global actions
        self.env_sync.initialize()

        # 9. Initialize ActorManager and spawn/attach Ego & initial NPCs
        self.actor_manager = ActorManager(
            esmini=self.esmini,
            carla_client=self.client,
            carla_world=self.world,
            ego_name=self.config.ego_name,
            ego_role_name=self.config.ego_role_name,
            ego_blueprint=self.config.ego_blueprint,
            spawn_ego=self.config.spawn_ego,
            wait_for_ego=self.config.wait_for_ego,
            ego_wait_timeout=self.config.ego_wait_timeout,
            teleport_external_ego_to_init=self.config.teleport_external_ego,
            keep_ego_on_exit=self.config.keep_ego_on_exit,
            enable_ros2=self.config.enable_ros2,
            autoware_mode=self.config.autoware_mode,
            autoware_sensors=self.config.autoware_sensors,
            apply_center_offset=self.config.apply_center_offset,
            draw_ghost=self.config.draw_ghost,
            carla_mod=self.carla_mod,
        )
        self.actor_manager.initialize_actors()

        # Perform one initial CARLA tick so spawned actors settle
        if self.world is not None:
            self._advance_carla_world()
            if self.config.follow_ego:
                self.actor_manager.update_spectator()

        self._is_setup = True

    def _apply_parameter_overrides(self) -> None:
        for param_name, param_val in self.config.parameters.items():
            self.esmini.set_parameter_auto(param_name, param_val)

    def _on_condition_triggered(self, name: str, timestamp: float) -> None:
        self.triggered_conditions.append((name, timestamp))

    def _on_storyboard_state_change(
        self,
        name: str,
        elem_type: SEStoryBoardElementType,
        state: SEStoryBoardElementState,
        full_path: str,
    ) -> None:
        self.storyboard_transitions.append((name, elem_type, state, full_path))
        if self.env_sync is not None:
            self.env_sync.on_storyboard_state_change(name, elem_type, state, full_path)

    # ------------------------------------------------------------------
    # Stepping & Execution Loop
    # ------------------------------------------------------------------

    def step(self, dt: float | None = None) -> bool:
        """Execute a single co-simulation step.

        Returns ``True`` if the scenario is still running, or ``False`` if ``SE_GetQuitFlag()``
        or ``max_sim_time`` has been reached.
        """
        if not self._is_setup:
            self.setup()

        step_dt = float(dt if dt is not None else self.config.delta_seconds)

        if self.esmini.get_quit_flag() == 1:
            return False

        if (
            self.config.max_sim_time is not None
            and self.esmini.get_simulation_time() >= self.config.max_sim_time
        ):
            return False

        # 1. Read CARLA Ego state and report to esmini before stepping
        if self.actor_manager is not None:
            self.actor_manager.report_ego_to_esmini(dt=step_dt)

        # 2. Step esmini forward by step_dt
        res = self.esmini.step_dt(step_dt)
        if res != 0:
            return False

        # 3. Synchronize all NPC states from esmini to CARLA in a single RPC batch
        if self.actor_manager is not None:
            self.actor_manager.sync_npcs_from_esmini()
            if self.config.follow_ego:
                self.actor_manager.update_spectator()

        # 4. Invoke any user-registered step hooks
        sim_time = self.esmini.get_simulation_time()
        for hook in self.step_callbacks:
            hook(sim_time)

        # 5. Advance CARLA simulation
        if self.world is not None:
            self._advance_carla_world()

        return self.esmini.get_quit_flag() == 0

    def _advance_carla_world(self) -> None:
        if self.world is None:
            return
        if self.config.passive_tick:
            self.world.wait_for_tick()
        elif self.config.sync_mode:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def run(self) -> float:
        """Run the scenario to completion and guarantee cleanup on exit.

        Returns the total simulated scenario duration in seconds.
        """
        try:
            if not self._is_setup:
                self.setup()

            dt = float(self.config.delta_seconds)
            target_wall_dt = (
                dt / self.config.time_scale
                if self.config.time_scale and self.config.time_scale > 0.0
                else 0.0
            )

            while True:
                t_start = time.monotonic()
                keep_running = self.step(dt)
                if not keep_running:
                    break
                if target_wall_dt > 0.0 and not self.config.dry_run:
                    elapsed = time.monotonic() - t_start
                    if elapsed < target_wall_dt:
                        time.sleep(target_wall_dt - elapsed)

            return self.esmini.get_simulation_time()
        finally:
            self.cleanup()

    # ------------------------------------------------------------------
    # Cleanup / Teardown
    # ------------------------------------------------------------------

    def cleanup(self) -> None:
        """Clean up spawned CARLA actors, traffic lights, world settings, and esmini."""
        if self.actor_manager is not None:
            try:
                self.actor_manager.cleanup()
            except Exception:
                pass
            self.actor_manager = None

        if self.env_sync is not None:
            try:
                self.env_sync.cleanup()
            except Exception:
                pass
            self.env_sync = None

        try:
            self.esmini.close()
        except Exception:
            pass

        if self.world is not None and self._original_world_settings is not None:
            try:
                self.world.apply_settings(self._original_world_settings)
            except Exception:
                pass
            self._original_world_settings = None

        if self._temp_xodr_dir is not None and self._temp_xodr_dir.exists():
            shutil.rmtree(self._temp_xodr_dir, ignore_errors=True)
            self._temp_xodr_dir = None

        self._is_setup = False

    def __enter__(self) -> CarlaEsminiRunner:
        self.setup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.cleanup()
