#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${HOME}/ros_ws_jazzy"
clone_root="${workspace_root}/sentry_navgation_RM27"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as your normal WSL user; it will use sudo when needed." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != ubuntu || "${VERSION_ID}" != 24.04 ]]; then
  echo "Expected Ubuntu 24.04; found ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != amd64 ]]; then
  echo "Expected x86_64/amd64; found $(dpkg --print-architecture)." >&2
  exit 1
fi
if [[ -e "${clone_root}" \
    && ! -f "${clone_root}/.navi_setup_incomplete" \
    && ! -f "${clone_root}/.navi_setup_complete" ]]; then
  echo "Build workspace already exists: ${clone_root}" >&2
  echo "Move it aside intentionally before creating a replacement." >&2
  exit 1
fi

echo "Installing ROS 2 Jazzy and Gazebo Harmonic prerequisites..."
sudo -v
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe

ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
ros_apt_version="${ros_apt_version#v}"
case "${ros_apt_version}" in
  *[!0-9.]*|'')
    echo "Unexpected ros-apt-source version: ${ros_apt_version}" >&2
    exit 1
    ;;
esac
ros_apt_deb="/tmp/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb"
curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb" \
  -o "${ros_apt_deb}"
sudo dpkg -i "${ros_apt_deb}"
rm -f "${ros_apt_deb}"
sudo sed -i \
  's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|' \
  /usr/share/ros-apt-source/ros2.sources
sudo sed -i 's/^Types: deb deb-src$/Types: deb/' \
  /usr/share/ros-apt-source/ros2.sources

sudo install -d -m 0755 /usr/share/keyrings
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  libeigen3-dev libflann-dev libomp-dev \
  python3-colcon-common-extensions python3-rosdep python3-vcstool ros-dev-tools \
  ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-slam-toolbox ros-jazzy-ros-gz \
  gz-harmonic

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "Creating an isolated build workspace with Jazzy-compatible Nav2 plugin revisions..."
mkdir -p "${workspace_root}"
if [[ -f "${clone_root}/.navi_setup_complete" ]]; then
  mv "${clone_root}/.navi_setup_complete" "${clone_root}/.navi_setup_incomplete"
fi
if [[ ! -d "${clone_root}/.git" ]]; then
  git clone --local --no-hardlinks "${repo_root}" "${clone_root}"
  touch "${clone_root}/.navi_setup_incomplete"
else
  source_commit="$(git -C "${repo_root}" rev-parse HEAD)"
  build_commit="$(git -C "${clone_root}" rev-parse HEAD)"
  if [[ "${source_commit}" != "${build_commit}" ]]; then
    git -C "${clone_root}" fetch --quiet --no-tags "${repo_root}" HEAD
    git -C "${clone_root}" reset --hard FETCH_HEAD
  fi
fi
for module in \
  src/livox_ros_driver2 \
  src/pb_nav2_plugins \
  src/pb_omni_pid_pursuit_controller \
  src/pb_teleop_twist_joy \
  src/point_lio \
  src/pointcloud_to_laserscan; do
  expected_head="$(git -C "${repo_root}/${module}" rev-parse HEAD)"
  case "${module}" in
    src/pb_nav2_plugins|src/pb_omni_pid_pursuit_controller)
      # These checked-out submodule revisions target the older Nav2 API.
      # Their local yhong refs contain the corresponding Jazzy migration.
      if ! git -C "${repo_root}/${module}" show-ref --quiet refs/remotes/origin/yhong; then
        echo "Missing Jazzy migration ref origin/yhong in ${module}." >&2
        exit 1
      fi
      expected_head="$(git -C "${repo_root}/${module}" rev-parse origin/yhong)"
      ;;
  esac
  existing_head=""
  if [[ -e "${clone_root}/${module}/.git" ]]; then
    existing_head="$(git -C "${clone_root}/${module}" rev-parse HEAD 2>/dev/null || true)"
  fi
  if [[ "${existing_head}" == "${expected_head}" && -f "${clone_root}/${module}/package.xml" ]]; then
    continue
  fi
  if [[ -e "${clone_root}/${module}" ]]; then
    mkdir -p "${clone_root}/.incomplete-modules/$(dirname "${module}")"
    mv "${clone_root}/${module}" "${clone_root}/.incomplete-modules/${module}"
  fi
  git clone --local --no-hardlinks "${repo_root}/${module}" "${clone_root}/${module}"
  if [[ "${module}" == src/pb_nav2_plugins || "${module}" == src/pb_omni_pid_pursuit_controller ]]; then
    git -C "${clone_root}/${module}" fetch --quiet \
      "${repo_root}/${module}" \
      refs/remotes/origin/yhong:refs/remotes/origin/yhong
  fi
  # The source checkout may have staged deletions inside a submodule. Keep
  # the build copy on the checked-out commit while leaving the user's checkout
  # untouched.
  git -C "${clone_root}/${module}" reset --hard "${expected_head}"
done
git -C "${clone_root}/src/point_lio" submodule update --init --recursive

echo "Installing package.xml dependencies for ROS 2 Jazzy..."
sudo -v
rosdep install --from-paths "${clone_root}/src" --ignore-src --rosdistro jazzy -r -y

echo "Building the workspace..."
cd "${clone_root}"
# WSL appends Windows paths under /mnt to PATH. CMake probes executables in
# PATH, and those probes can stall on the 9P mount, so omit them for this build.
build_path="$(python3 -c 'import os; print(":".join(p for p in os.environ["PATH"].split(":") if not p.startswith("/mnt/")))')"
PATH="${build_path}" colcon build --symlink-install --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
mv "${clone_root}/.navi_setup_incomplete" "${clone_root}/.navi_setup_complete"

if ! grep -Fq '# >>> navi ROS 2 Jazzy workspace >>>' "${HOME}/.bashrc"; then
  cat >>"${HOME}/.bashrc" <<'BASHRC'

# >>> navi ROS 2 Jazzy workspace >>>
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/ros_ws_jazzy/sentry_navgation_RM27/install/setup.bash" ]; then
  source "$HOME/ros_ws_jazzy/sentry_navgation_RM27/install/setup.bash"
fi
# <<< navi ROS 2 Jazzy workspace <<<
BASHRC
fi

echo
echo "Setup and build completed. Open a new WSL terminal, then run:"
echo "  ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 slam:=False gui:=true use_rviz:=true hardware_acceleration:=false rviz_hardware_acceleration:=false"
