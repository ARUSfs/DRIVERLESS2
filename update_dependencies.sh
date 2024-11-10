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
echo -e "Made by Ángel García. Last update: 10-11-2024\n"
echo -e "${NC}"

# Selection menu
echo -e "Select the option you want to install:\n"
echo -e "${GREEN}1) ${NC}Install dependencies"
echo -n "Insert the number of the option you want to install: "
read option

# Function to install pip dependencies
install_pip_dependencies() {
  echo -e "${GREEN}Installing pip dependencies...${NC}"
  if pip3 install -r requirements_pip.txt &> /dev/null; then
    echo -e "${GREEN}Pip dependencies installed successfully.${NC}"
  else
    echo -e "${RED}Error installing pip dependencies.${NC}"
  fi
}

# Function to install apt dependencies
install_apt_dependencies() {
  echo -e "${GREEN}Installing apt dependencies...${NC}"
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
    echo -e "${YELLOW}WARNING: The commit-msg hook already exists. No changes were made.${NC}"
    return
  fi

  cat > "$HOOK_PATH" << 'EOF'
#!/bin/sh

commit_message=$(cat "$1")

if ! echo "$commit_message" | grep -Eq '^(feat|fix|docs|test|chore|ci|style|refactor|revert): [a-z]+ '; then
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

# Always configure the commit-msg hook
configure_commit_msg_hook

# Process selected option
case $option in
  1)
    install_dependencies
    ;;
  *)
    echo -e "${RED}Invalid option. No dependencies installed.${NC}"
    ;;
esac

