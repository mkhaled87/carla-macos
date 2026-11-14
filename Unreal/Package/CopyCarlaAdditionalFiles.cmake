
# The archive directory is reused between builds and the copies below only add files:
# clear what this script owns, so files of earlier builds (old wheels, removed
# examples, older DLSS snippets) never reach a new package.
file (REMOVE_RECURSE
  ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI
  ${CARLA_PACKAGE_ARCHIVE_PATH}/Tools
  ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Plugins/Carla/Shaders)
file (GLOB STALE_DLSS_SNIPPETS
  ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Binaries/*/libnvidia-ngx-*
  ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Binaries/*/nvngx_*.dll
  ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Binaries/*/NVIDIA_DLSS_LICENSE.txt)
if (STALE_DLSS_SNIPPETS)
  file (REMOVE ${STALE_DLSS_SNIPPETS})
endif ()

file (COPY_FILE ${CARLA_WORKSPACE_PATH}/LICENSE ${CARLA_PACKAGE_ARCHIVE_PATH}/LICENSE)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/CHANGELOG.md ${CARLA_PACKAGE_ARCHIVE_PATH}/CHANGELOG)
# One README per platform: the commands differ (CarlaUnreal.sh / CarlaUnreal.exe, python3 / python).
if (WIN32)
  file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Docs/release_readme_windows.md ${CARLA_PACKAGE_ARCHIVE_PATH}/README)
else ()
  file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Docs/release_readme.md ${CARLA_PACKAGE_ARCHIVE_PATH}/README)
endif ()
# The release Docker image is built from the package root:
#   docker build -t carlasim/carla:<version> .
if (NOT WIN32)
  file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Util/Docker/Release.Dockerfile ${CARLA_PACKAGE_ARCHIVE_PATH}/Dockerfile)
endif ()
if (WIN32)
    set (EXE_EXT .exe)
    set (RECASTBUILDER_SRC ${CARLA_BUILD_PATH}/_deps/recastnavigation-build/RecastBuilder/RecastBuilder.exe)
    set (RECASTBUILDER_DST ${CARLA_PACKAGE_ARCHIVE_PATH}/Tools/RecastBuilder.exe)
elseif (APPLE)
    set (EXE_EXT)
    set (RECASTBUILDER_SRC ${CARLA_BUILD_PATH}/_deps/recastnavigation-build/RecastBuilder/RecastBuilder.app/Contents/MacOS/RecastBuilder)
    set (RECASTBUILDER_DST ${CARLA_PACKAGE_ARCHIVE_PATH}/Tools/RecastBuilder)
else ()
    set (EXE_EXT)
    set (RECASTBUILDER_SRC ${CARLA_BUILD_PATH}/_deps/recastnavigation-build/RecastBuilder/RecastBuilder)
    set (RECASTBUILDER_DST ${CARLA_PACKAGE_ARCHIVE_PATH}/Tools/RecastBuilder)
endif ()
make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/Tools)
# @TODO Temporary hack:
# The packaged binary looks for RecastBuilder at FPaths::RootDir()/Tools, the
# archive root once FlattenPackageLayout.cmake has run (CarlaEpisode.cpp
# BuildRecastBuilderFile).
file (COPY_FILE ${RECASTBUILDER_SRC} ${RECASTBUILDER_DST})
# carla-pack installs content packs (AdditionalMaps) into this package; standard library only.
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Util/ContentPacks/carla_pack.py ${CARLA_PACKAGE_ARCHIVE_PATH}/Tools/carla_pack.py)
# FCarlaModule::AddShaderSearchPaths maps /Plugin/Carla to <plugin>/Shaders at
# startup; the plugin is staged without its Shaders folder, and the shader-based
# sensors need the .usf/.ush sources at runtime.
cmake_path (GET CARLA_UE_PROJECT_PATH PARENT_PATH CARLA_UE_PROJECT_DIR)
if (APPLE)
  file (GLOB CARLA_PACKAGE_MAC_APP_DIRS LIST_DIRECTORIES true
    ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal*.app
    ${CARLA_PACKAGE_ARCHIVE_PATH}/*/CarlaUnreal*.app)
  foreach (CARLA_PACKAGE_APP_DIR ${CARLA_PACKAGE_MAC_APP_DIRS})
    make_directory (${CARLA_PACKAGE_APP_DIR}/Contents/UE/Tools)
    file (COPY_FILE ${RECASTBUILDER_SRC} ${CARLA_PACKAGE_APP_DIR}/Contents/UE/Tools/RecastBuilder)
    file (
      COPY ${CARLA_UE_PROJECT_DIR}/Plugins/Carla/Shaders/
      DESTINATION ${CARLA_PACKAGE_APP_DIR}/Contents/UE/CarlaUnreal/Plugins/Carla/Shaders/)
  endforeach ()
else ()
  file (
    COPY ${CARLA_UE_PROJECT_DIR}/Plugins/Carla/Shaders/
    DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Plugins/Carla/Shaders/)
endif ()

if (APPLE)
  # Update staged app bundle with freshly compiled Shipping binary if present
  set (FRESH_SHIPPING_BIN "${CARLA_WORKSPACE_PATH}/Unreal/CarlaUnreal/Binaries/Mac/CarlaUnreal-Mac-Shipping.app/Contents/MacOS/CarlaUnreal-Mac-Shipping")
  file (GLOB CARLA_MAC_APP_BUNDLES LIST_DIRECTORIES true
    ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal*.app
    ${CARLA_PACKAGE_ARCHIVE_PATH}/*/CarlaUnreal*.app)
  if (EXISTS "${FRESH_SHIPPING_BIN}")
    foreach (CARLA_MAC_APP ${CARLA_MAC_APP_BUNDLES})
      if (EXISTS "${CARLA_MAC_APP}/Contents/MacOS/CarlaUnreal-Mac-Shipping")
        file (COPY_FILE "${FRESH_SHIPPING_BIN}" "${CARLA_MAC_APP}/Contents/MacOS/CarlaUnreal-Mac-Shipping")
      endif ()
    endforeach ()
  endif ()

  # Re-sign the staged macOS app bundles without sandbox entitlements.
  # Unreal Engine's default shipping config applies Sandbox.NoNet.entitlements,
  # which enables com.apple.security.app-sandbox without network permissions,
  # causing bind() to fail with "Operation not permitted".
  foreach (CARLA_MAC_APP ${CARLA_MAC_APP_BUNDLES})
    execute_process (
      COMMAND /usr/bin/codesign --force --sign - ${CARLA_MAC_APP}
      RESULT_VARIABLE CODESIGN_RESULT
    )
    if (CODESIGN_RESULT EQUAL 0)
      message (STATUS "Re-signed ${CARLA_MAC_APP} without app-sandbox restrictions.")
    else ()
      message (WARNING "Failed to re-sign ${CARLA_MAC_APP} (${CODESIGN_RESULT}).")
    endif ()
  endforeach ()

  file (WRITE ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal.sh
"#!/bin/bash
SCRIPT_DIR=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")\" && pwd)\"
# Normalize space-separated port flags (-carla-rpc-port 2000 -> -carla-rpc-port=2000)
ARGS=()
while [[ \$# -gt 0 ]]; do
  case \"\$1\" in
    -carla-rpc-port|-carla-port|-world-port|-carla-streaming-port|-carla-secondary-port)
      if [[ -n \"\$2\" && \"\$2\" != -* ]]; then
        ARGS+=(\"\$1=\$2\")
        shift 2
      else
        ARGS+=(\"\$1\")
        shift
      fi
      ;;
    *)
      ARGS+=(\"\$1\")
      shift
      ;;
  esac
done
if [[ -x \"\$SCRIPT_DIR/CarlaUnreal-Mac-Shipping.app/Contents/MacOS/CarlaUnreal-Mac-Shipping\" ]]; then
  exec \"\$SCRIPT_DIR/CarlaUnreal-Mac-Shipping.app/Contents/MacOS/CarlaUnreal-Mac-Shipping\" \"\${ARGS[@]}\"
fi
APP_BIN=\$(find \"\$SCRIPT_DIR\" -maxdepth 4 -path \"*CarlaUnreal*.app/Contents/MacOS/CarlaUnreal*\" -perm +111 -type f 2>/dev/null | head -n 1)
if [[ -z \"\$APP_BIN\" ]]; then
  echo \"ERROR: Could not find CarlaUnreal*.app binary under \$SCRIPT_DIR\" >&2
  exit 1
fi
exec \"\$APP_BIN\" \"\${ARGS[@]}\"
")
  execute_process (COMMAND chmod +x ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal.sh)
endif ()

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/carla/dist)
# Every wheel the build made, one per Python version in CARLA_PYTHON_VERSIONS:
if (NOT CARLA_PYTHON_API_DIST_PATH)
  set (CARLA_PYTHON_API_DIST_PATH ${CARLA_BUILD_PATH}/PythonAPI/dist)
endif ()
# Only this version's wheels: dist/ keeps wheels of earlier versions built in the same tree.
file (GLOB PYTHON_WHL_FILES ${CARLA_PYTHON_API_DIST_PATH}/carla-${CARLA_VERSION}-*.whl)
if (NOT PYTHON_WHL_FILES)
  message (FATAL_ERROR "No carla-${CARLA_VERSION} wheel in ${CARLA_PYTHON_API_DIST_PATH}")
endif ()
file (COPY ${PYTHON_WHL_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/carla/dist/)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Docs/python_api.md ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/python_api.md)
file (COPY ${CARLA_WORKSPACE_PATH}/PythonAPI/carla/agents/ DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/carla/agents/)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/PythonAPI/carla/scene_layout.py ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/carla/scene_layout.py)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/PythonAPI/carla/requirements.txt ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/carla/requirements.txt)

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/)
file (GLOB PYTHON_EXAMPLE_FILES ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/*.py ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/*.xml)
file (COPY ${PYTHON_EXAMPLE_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/)
# make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/rss/)
# file (GLOB PYTHON_EXAMPLE_RSS_FILES ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/rss/*.py)
# file (COPY ${PYTHON_EXAMPLE_RSS_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/rss/)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/requirements.txt ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/requirements.txt)

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/ros2)
file (GLOB PYTHON_EXAMPLE_ROS2_FILES ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/ros2/*)
file (COPY ${PYTHON_EXAMPLE_ROS2_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/ros2)
make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/ros2/config)
file (GLOB PYTHON_EXAMPLE_CONFIG_ROS2_FILES ${CARLA_WORKSPACE_PATH}/PythonAPI/examples/ros2/config/*)
file (COPY ${PYTHON_EXAMPLE_CONFIG_ROS2_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/examples/ros2/config)

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/esmini)
file (COPY ${CARLA_WORKSPACE_PATH}/PythonAPI/esmini/ DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/esmini/)

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/util/)
file (GLOB PYTHON_UTIL_FILES ${CARLA_WORKSPACE_PATH}/PythonAPI/util/*.py)
file (COPY ${PYTHON_UTIL_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/util/)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/PythonAPI/util/requirements.txt ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/util/requirements.txt)
file (COPY ${CARLA_WORKSPACE_PATH}/PythonAPI/util/opendrive/ DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/util/opendrive/)

# Example folders the docs and CHANGELOG point at. Only git-tracked files: these
# trees also hold local models, venvs and outputs that must not be packaged.
set (CARLA_PACKAGE_EXAMPLE_TREES
  PythonAPI/examples/nvidia
  PythonAPI/examples/av_stacks
  PythonAPI/util/tools)
find_package (Git QUIET)
if (GIT_FOUND AND EXISTS ${CARLA_WORKSPACE_PATH}/.git)
  execute_process (
    COMMAND ${GIT_EXECUTABLE} -C ${CARLA_WORKSPACE_PATH} ls-files -- ${CARLA_PACKAGE_EXAMPLE_TREES}
    OUTPUT_VARIABLE CARLA_PACKAGE_EXAMPLE_FILES
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
  string (REPLACE "\n" ";" CARLA_PACKAGE_EXAMPLE_FILES "${CARLA_PACKAGE_EXAMPLE_FILES}")
  foreach (EXAMPLE_FILE ${CARLA_PACKAGE_EXAMPLE_FILES})
    cmake_path (GET EXAMPLE_FILE PARENT_PATH EXAMPLE_DIR)
    file (COPY ${CARLA_WORKSPACE_PATH}/${EXAMPLE_FILE} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/${EXAMPLE_DIR})
  endforeach ()
else ()
  message (WARNING "Not a git checkout: ${CARLA_PACKAGE_EXAMPLE_TREES} are not packaged.")
endif ()

# NGX loads the DLSS / DLSS-RR model snippets from the executable's folder; without
# them DLSS upscaling and the rt_lens denoiser silently fall back.
if (CARLA_DLSS_SDK_RESOLVED AND NOT CARLA_DLSS_SDK_RESOLVED STREQUAL "disabled")
  if (WIN32)
    set (DLSS_SNIPPET_GLOBS
      ${CARLA_DLSS_SDK_RESOLVED}/lib/Windows_x86_64/rel/nvngx_dlss.dll
      ${CARLA_DLSS_SDK_RESOLVED}/lib/Windows_x86_64/rel/nvngx_dlssd.dll)
    set (DLSS_SNIPPET_DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Binaries/Win64)
  else ()
    set (DLSS_SNIPPET_GLOBS
      ${CARLA_DLSS_SDK_RESOLVED}/lib/Linux_x86_64/rel/libnvidia-ngx-dlss.so.*
      ${CARLA_DLSS_SDK_RESOLVED}/lib/Linux_x86_64/rel/libnvidia-ngx-dlssd.so.*)
    set (DLSS_SNIPPET_DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/CarlaUnreal/Binaries/Linux)
  endif ()
  file (GLOB DLSS_SNIPPETS ${DLSS_SNIPPET_GLOBS})
  list (LENGTH DLSS_SNIPPETS DLSS_SNIPPET_COUNT)
  if (NOT DLSS_SNIPPET_COUNT EQUAL 2)
    message (FATAL_ERROR "Expected the DLSS and DLSS-RR snippets in ${CARLA_DLSS_SDK_RESOLVED}, found: ${DLSS_SNIPPETS}")
  endif ()
  file (COPY ${DLSS_SNIPPETS} DESTINATION ${DLSS_SNIPPET_DESTINATION})
  if (EXISTS ${CARLA_DLSS_SDK_RESOLVED}/LICENSE.txt)
    file (COPY_FILE ${CARLA_DLSS_SDK_RESOLVED}/LICENSE.txt ${DLSS_SNIPPET_DESTINATION}/NVIDIA_DLSS_LICENSE.txt)
  endif ()
endif ()

make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/Util/MapBuilder/)
file (GLOB MAPBUILDER_FILES ${CARLA_WORKSPACE_PATH}/Util/MapBuilder/*)
file (COPY ${MAPBUILDER_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/Util/MapBuilder/)
file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Util/MapBuilder/carla_map.py ${CARLA_PACKAGE_ARCHIVE_PATH}/PythonAPI/util/carla_map.py)

# Top-level carla-map executable wrapper
file (WRITE ${CARLA_PACKAGE_ARCHIVE_PATH}/carla-map
"#!/usr/bin/env bash
SCRIPT_DIR=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")\" && pwd)\"

if command -v python3.10 &>/dev/null; then
    PY_BIN=\"python3.10\"
elif command -v python3 &>/dev/null; then
    PY_BIN=\"python3\"
else
    echo \"ERROR: python3 not found\" >&2
    exit 1
fi

WHL=\$(find \"\$SCRIPT_DIR/PythonAPI/carla/dist\" -name \"carla-*.whl\" 2>/dev/null | head -n 1)
if [[ -n \"\$WHL\" ]]; then
    export PYTHONPATH=\"\$WHL:\$SCRIPT_DIR:\$SCRIPT_DIR/Util/MapBuilder:\$PYTHONPATH\"
else
    export PYTHONPATH=\"\$SCRIPT_DIR:\$SCRIPT_DIR/Util/MapBuilder:\$PYTHONPATH\"
fi

if [[ -d \"\$SCRIPT_DIR/Tools\" ]]; then
    export PATH=\"\$SCRIPT_DIR/Tools:\$PATH\"
fi
if [[ -d \"\$SCRIPT_DIR/Mac/Tools\" ]]; then
    export PATH=\"\$SCRIPT_DIR/Mac/Tools:\$PATH\"
fi

exec \"\$PY_BIN\" \"\$SCRIPT_DIR/Util/MapBuilder/carla_map.py\" \"\$@\"
")
execute_process (COMMAND chmod +x ${CARLA_PACKAGE_ARCHIVE_PATH}/carla-map)
execute_process (COMMAND chmod +x ${CARLA_PACKAGE_ARCHIVE_PATH}/Util/MapBuilder/carla-map)

# file (COPY ${CARLA_WORKSPACE_PATH}/Co-Simulation/ DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/Co-Simulation/)

# make_directory (${CARLA_PACKAGE_ARCHIVE_PATH}/HDMaps/)
# file (GLOB PYTHON_HDMAP_FILES ${CARLA_WORKSPACE_PATH}/Unreal/CarlaUnreal/Content/Carla/HDMaps/*.pcd)
# file (COPY ${PYTHON_HDMAP_FILES} DESTINATION ${CARLA_PACKAGE_ARCHIVE_PATH}/HDMaps/)
# file (COPY_FILE ${CARLA_WORKSPACE_PATH}/Unreal/CarlaUnreal/Content/Carla/HDMaps/Readme.md ${CARLA_PACKAGE_ARCHIVE_PATH}/HDMaps/README)
