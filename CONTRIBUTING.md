The RadioHead library is maintained at:

http://www.airspayce.com/mikem/arduino/RadioHead/

by Mike McCauley. DO NOT CONTACT THE AUTHOR DIRECTLY. 
USE THE GOOGLE LIST: https://groups.google.com/forum/#!forum/radiohead-arduino

This document is intended to help users understand how to contribute to this project.

The best way to contribute is to send a context diff, also called a "unified diff file", suitable for use with patch. If the changes are generally useful, and consistent with the rest of the code, the author will consider adopting them.

Many developers are using GitHub. Fortunately it has a command-line feature to generate the proper diff file for the maintainer.

Adafruit created a github'ified version here:

https://github.com/adafruit/RadioHead

but is not actively merged with updates from the official RadioHead site. It does however have over a dozen forks which may or may have valuable changes that may or may not find their way back to the official source mentioned above.

When using GitHub, a diff is created for every commit. The maintainers are not really interested in each increment - and thus is is best to have a single proposed diff file for all changes.

If you already have a GitHub respository, it would be best to either (1) create a new, empty branch or (2) create a new repository.

# New Branch:
1. create a new branch
2. delete everything (use with caution!)
3. commit
4. copy the latest zip file version
5. commit
6. copy your latest code
7. commit
8. create a diff file 

Specifically:

1. Create a New Branch (probably easiest with the wb interface: simply click the dropdown and create a new branch, in this case called AirSpayce.

Go to your contribution workspace directory and clone the respository, for example (be careful not to use your normal github working directory!):

```
cd ~/workspace-PR/
git clone https://github.com/gojimmypi/RadioHead.git
cd RadioHead
git checkout AirSpayce
git branch --list
```

2. Then remove everything except the hidden .git directory:

```
rm -r examples
rm -r RHutil
rm -r STM32ArduinoCompat
rm -r tools
rm *.cpp
rm *.h
```

3. Commit the emptied directory:

```
git commit -a -m "purge existing files in prep for zip"
git push
git status
```

4. Get the target file from RadioHead. Note that the zip file contains a directory called RadioHead. Unzip the file to the workspace directory. Note from commandline, it is easiest if the zip file is in the parent directory (the same place that git clone is used). 

```
cd ..
unzip RadioHead-1.83.zip
cd RadioHead
```

5. And commit these changes:

```
git add .
git commit -a -m "Load RadioHead-1.83.zip"
git push
git status
```

The AirSpayce branch now contains the latest official RadioHead code (in this example, 1.83). You may wish to browse the web site to confirm. In this example:

https://github.com/gojimmypi/RadioHead/tree/AirSpayce

Note that there's now a clean commit of the RadioHead source. 

6. Proceed to copy all the files you'd like to contribute from your main development working directory. Be sure to *NOT COPY* the hidden .git directory! You probably want to also exclude the Debug, Release, .vs and __vm directories (if you use Visual Studio) and exclude the .vscode directory (if you use VS Code)

7. commit your changes

```
git add .
git commit -a -m "Contribution from Adafruit fork and gojimmypi changes"
git push
git status
```

8. Create adiff file with for use with patch:

```
git format-patch -1
```

Note the "-1" indicates that you want only 1 diff file for the most recent one commit (from the fresh RadioHead to current code of your code()

in this case a file should have been created, called something like

```
0001-contribution-from-Adafruit-fork-and-gojimmypi-changes.patch
```

Simply send this file off to Mike for consideration.

In the future, you'd only need to do steps 7 and 8 to copy your latest code and generate the diff file.

Thanks for contributing to RadioHead!






Copy the zip file contents

# New Respository:
1. create a new respository
2. copy the latest zip file version
3. commit
4. copy your latest code
5. commit
6. create a diff file 



