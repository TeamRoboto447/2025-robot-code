#!/bin/bash
declare -a arr=("ryan" "ronen" "connor" "gideon" "group-dev")

for i in "${arr[@]}"
do
    echo "Rebasing $i's branch with code from main"
    git checkout $i
    git rebase origin/main
    git push
done
git checkout main