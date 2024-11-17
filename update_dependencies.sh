#!/bin/bash

# Define colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
NC='\033[0m' # No color

# Banner
cat << "EOF"


     _    ____  _   _ ____    ____       _                _                 ____
    / \  |  _ \| | | / ___|  |  _ \ _ __(_)_   _____ _ __| | ___  ___ ___  |___ \
   / _ \ | |_) | | | \___ \  | | | | '__| \ \ / / _ \ '__| |/ _ \/ __/ __|   __) |
  / ___ \|  _ <| |_| |___) | | |_| | |  | |\ V /  __/ |  | |  __/\__ \__ \  / __/
 /_/   \_\_| \_\\___/|____/  |____/|_|  |_| \_/ \___|_|  |_|\___||___/___/ |_____|



EOF

# Welcome menu
echo -e "${BLUE}Welcome to the ARUS Driverless2 dependencies installer.${NC}"
echo -e "This script will install all the dependencies needed to run the ARUS Driverless2 project."
echo -e "This script was tested on Ubuntu 22.04.2 LTS (Jammy Jellyfish)"
echo -e "Made by Ángel García. Last update: 16-11-2024\n"
echo -e "${NC}"

# Selection menu
echo -e "Select the option you want to install:\n"
echo -e "${GREEN}1) ${NC}Install dependencies"
echo -e "${GREEN}2) ${NC}Install ROS2 Humble"

echo -n "\nInsert the number of the option you want to install: "
read option

# Function to install pip dependencies
install_pip_dependencies() {
  echo -e "${NC}Installing pip dependencies...${NC}"
  if pip3 install -r requirements_pip.txt &> /dev/null; then
    echo -e "${GREEN}Pip dependencies installed successfully.${NC}"
  else
    echo -e "${RED}Error installing pip dependencies.${NC}"
  fi
}

# Function to install apt dependencies
install_apt_dependencies() {
  echo -e "${NC}Installing apt dependencies...${NC}"
  sudo apt update -y &> /dev/null
  if xargs -a requirements_apt.txt sudo apt install -y &> /dev/null; then
    echo -e "${GREEN}Apt dependencies installed successfully.${NC}"
  else
    echo -e "${RED}Error installing apt dependencies.${NC}"
  fi
}

# Configure git commit-msg hook
configure_commit_msg_hook() {
  if [ ! -d ".git" ]; then
    echo -e "${RED}ERROR: You are not in the root directory of a Git repository.${NC}"
    exit 1
  fi

  HOOK_PATH=".git/hooks/commit-msg"
  if [ -f "$HOOK_PATH" ]; then
    rm "$HOOK_PATH"
  fi

  cat > "$HOOK_PATH" << 'EOF'
#!/bin/sh

commit_message=$(cat "$1")

if ! echo "$commit_message" | grep -Eq '^(feat|fix|docs|test|chore|ci|style|refactor|revert): [a-z]+ |^Merge '; then
  echo "\033[0;31mERROR: The commit message does not follow the required format:\033[0m"
  echo "Format: <type>: <imperative_verb> message content"
  echo "Where type can be: \033[1;33mfeat, fix, docs, test, chore, ci, style, refactor, revert\033[0m"
  echo "Example: \033[1;33mfeat: create triangulation path\033[0m"
  exit 1
fi
EOF

  chmod +x "$HOOK_PATH"
  echo -e "${GREEN}commit-msg hook successfully created in .git/hooks/commit-msg.${NC}"
}

# Main function to install dependencies
install_dependencies() {
  install_pip_dependencies
  install_apt_dependencies
}

# Function to install ROS2 Humble
install_ros2_humble() {
  echo -e "${NC}Installing ROS2 Humble...${NC}"

  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  sudo apt install software-properties-common -y
  sudo add-apt-repository universe

  sudo apt update -y && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update -y && sudo apt upgrade
  sudo apt install ros-humble-desktop -y

  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

  echo -e "${GREEN}ROS2 Humble installed successfully.${NC}"
}

# Always configure the commit-msg hook
configure_commit_msg_hook

# Process selected option
case $option in
  1)
    install_dependencies
    ;;
  2)
    install_ros2_humble
    ;;
  *)
    echo -e "${RED}Invalid option. No dependencies installed.${NC}"
    ;;
esac
