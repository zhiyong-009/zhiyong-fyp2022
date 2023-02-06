#!/bin/bash
source <(curl -s https://gitlab.com/VictorLamoine/bloom_local_release/raw/master/bloom_local_release.bash)

# This custom version deletes svgpathtools dependency in debian/control file.
# Generate a debian package from a ROS package.
# $os, $os_version, $ros_distro must be defined before calling.
# Arguments:
# $1 = Package name (package-name with dashes, no underscores!)
# $2 = CMake additional arguments (eg: '-Dmy_arg="true"')
# $3 = Ignore missing info if set to true
generate_deb_ignore_svgpathtools()
{
  echo -e "\033[34m--------------------------------------------"
  echo -e "--------------------------------------------"
  echo -e "--------------------------------------------\033[39m"
  underscore_name="${1//\-/\_}"
  cd $underscore_name
  bloom-generate rosdebian --os-name $os --os-version $os_version --ros-distro $ros_distro
  : # Necessary otherwise the script "continues too fast"

  if [ ! -z "$2" ];
  then
    sed -i "/dh_auto_configure --/c\\\tdh_auto_configure -- ${2} \\\\" debian/rules
  fi

  if $3;
  then
    sed -i 's/dh_shlibdeps /dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info /g' debian/rules
  fi

  # Delete svgpathtools dependency
  sed -i 's/, svgpathtools//g' debian/control

  fakeroot debian/rules binary
  rm -rf debian/ obj-x86_64-linux-gnu/
  cd ..
  echo -e "\033[34m--------------------------------------------"
  echo -e "--------------------------------------------"
  echo -e "--------------------------------------------\033[39m"
}

os=ubuntu
os_version=bionic
ros_distro=melodic
rosdep_yaml_name="ros_additive_manufacturing"
rosdep_list_name="31-ros-additive-manufacturing"

# Check if dependencies are installed
check_deb_installed industrial-robot-angle-conversions

# Add rosdep rules we will need later when installing debian packages
append_rosdep_key industrial-robot-angle-conversions
append_rosdep_key ram-msgs
append_rosdep_key ram-utils
append_rosdep_key ram-modify-trajectory
append_rosdep_key ram-display
append_rosdep_key ram-post-processor
append_rosdep_key ram-path-planning
append_rosdep_key ram-trajectory
rosdep update

# Generate debians and install them
generate_deb ram-documentation
install_deb ram-documentation

generate_deb ram-msgs
install_deb ram-msgs

generate_deb_ignore_svgpathtools ram-utils "" true # VTK CPack debian misses some information
install_deb ram-utils

generate_deb ram-modify-trajectory
install_deb ram-modify-trajectory

generate_deb ram-display
install_deb ram-display

generate_deb ram-post-processor
install_deb ram-post-processor

generate_deb ram-trajectory
install_deb ram-trajectory

generate_deb ram-path-planning "" true # VTK CPack debian misses some information
install_deb ram-path-planning

generate_deb ram_qt_guis "" true # VTK CPack debian misses some information
install_deb ram-qt-guis

if [ "$1" == "" ]; then
  clear_rosdep_keys
fi

zip -r ros_additive_manufacturing *.deb install.bash
rm *.deb *.ddeb
installed_deb_info
