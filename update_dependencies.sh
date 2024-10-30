#!/bin/bash

# Define colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m' # Warning yellow
NC='\033[0m' # No color

# Check if we're in the root directory of a Git repository
if [ ! -d ".git" ]; then
  echo -e "${RED}ERROR: You are not in the root directory of a Git repository.${NC}"
  exit 1
fi

# Hook file path
HOOK_PATH=".git/hooks/commit-msg"

# Check if the hook already exists
if [ -f "$HOOK_PATH" ]; then
  echo -e "${YELLOW}WARNING: The commit-msg hook already exists. No changes were made.${NC}"
  exit 0
fi

# Create the hook file with the specified content
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

# Make the hook executable
chmod +x "$HOOK_PATH"

echo -e "${GREEN}commit-msg hook successfully created in .git/hooks/commit-msg.${NC}"
