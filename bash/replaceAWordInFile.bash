#!/bin/bash
if [ "$#" -ne 3 ]; then
    echo "USAGE:     ./replaceAWordInFile.bash path/to/file wordToReplace wordToReplaceWith"
    echo "EXAMPLE:   ls . -R"
    echo "           .:"	     
    echo "           fuckYou.bash  renameAllFilesInDirectory.bash  replaceAWordInEveryFileInDir.bash"
    echo "           cat fuckYou.bash"
    echo "           why?"
    echo "           ./replaceAWordInFile.bash fuckYou.bash why? because!"
else
    grep -r "$2" $1|column -c 100
    echo "-------"
    echo "WARNING: the content of the above files will be changed *$2* -> *$3*"
    echo "-------"
    read -r -p "Proceed? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
	datetimestring=$(date +'%d%m%Y_%H-%M')
        grep -r "$2" $1 >> /tmp/replaceAWordInFile_${datetimestring}.log
        grep -rl "$2" $1 | xargs sed -i "s/$2/$3/g"
        mv /tmp/replaceAWordInFile_${datetimestring}.log $1
    else	
        echo "ABORT"
    fi
fi

