#!/bin/sh
#args <first> <last> <user> <email> <token>
if [ $# -eq 5 ]; then
    ssh-keygen -t rsa -C "$4"
    echo "Copy and paste the following into your GitHub profile"
    echo "-----------------------------------------------------"
    cat ~/.ssh/id_rsa.pub
    echo "-----------------------------------------------------"
    echo "Press ENTER to Continue"
    read pause
    ssh -T git@github.com
    git config --global user.name "$1 $2"
    git config --global user.email "$4"
    git config --global github.user "$3"
    git config --global github.token "$5"
else
    echo "Syntax: <first> <last> <GitHub username> <email> <GitHub token>"
fi
