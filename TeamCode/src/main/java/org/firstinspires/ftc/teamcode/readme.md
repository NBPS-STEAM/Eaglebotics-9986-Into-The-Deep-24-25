# The SUPRA Readme

This is the readme file copied directly from SUPRA. Feel free to read.

# SUPRA

Welcome to SUPRA!

It's not necessary to read this whole file. To get started, at least read from here through to
the section on "Command-Based Programming". You may read beyond that for more information.
Past the "FTC Robot Controller Readme" header is the contents of the official FTC readme text.

From Eaglebotics (FTC teams 9986, 9987, 24000), with love

Written by Kelin Levine

## Get Started

The first step to writing robot code all by yourself is to [install Android Studio](https://developer.android.com/studio).
Alternatively, [install IntelliJ IDEA](https://www.jetbrains.com/idea/download) with the [Android and Android Design Tools plugins](https://www.jetbrains.com/help/idea/create-your-first-android-application.html).

Continue after you've done that and have this file open in the editor.

If you want to use SUPRA for your team, you should make a fork of this repository on GitHub,
then use Git to clone *the fork* to your computer. If you plan to make changes for your own team,
please don't clone or try to modify the original repository. It's the public one for everyone.

## Anatomy of FTC Robot Code

Robot code is written in a routine called an "operation mode", or opmode for short. An opmode
takes the form of a java file which contains code that is run when the opmode starts.

To run a robot, you select an opmode and run it.
A robot may have any number of opmodes installed, but can only run one at a time.
There are two types of opmodes:

* Tele-Operation (Teleop)
  * Robot may use controller inputs
  * Makes up the main portion of the match
* Autonomous
  * Robot may NOT use controller inputs
  * Makes up the first 30 seconds of the match

The robot works in two wirelessly connected parts: the Control Hub and the Driver Station.

* Control Hub: sits on the robot and runs code using attached motors and sensors.
  * A single Expansion Hub may be connected to the Control Hub to add more ports for motors and sensors.
* Driver Station: sits outside the playing field with the drivers
  * Communicates controller inputs
  * Signals the Control Hub to start or stop opmodes

For communication, the robot creates a wireless network when it turns on. You can connect using WiFi.
If connected from a driver station, you may configure the robot or run opmodes.
If connected from a computer, you may configure the robot or deploy new code.

When you deploy code, it installs as a single app containing all opmodes.
Deploying again will completely replace the previously installed app.
There is no undoing an install!

## Tele-Op Opmodes

Teleop routines typically have the driver(s) control the robot using remote controls. It's used
during the main portion of a match (the so-called teleoperated period).

It's recommended to program your teleop routines using command-based programming (see below).

## Autonomous Opmodes

INCOMPLETE, PLEASE COME BACK LATER

Make sure to check the competition manual for measurements!

## A (Very Brief) Intro to Java

INCOMPLETE, PLEASE COME BACK LATER

## Command-Based Programming

Command-based programming is a style of programming where you write code by configuring (binding)
actions (commands) to be performed when certain conditions are met (triggers).

i.e. When the B button on a controller (trigger) is pressed (binding), move the arm to high position (action).

To set up command-based programming, you should organize each section of the robot (i.e. drivetrain,
arm, etc.) into its own class file which extends 'SubsystemBase'. These are called subsystems.

When you bind a command, you have the option of assigning it a subsystem to occupy. If that command
should be run but another command is already running on that subsystem, then it will automatically
either prevent (block) itself from running or stop (interrupt) the other command.

You may also bind a default command. Default commands are called to run continuously while no other
commands are running on its subsystem. Default commands must be assigned a subsystem.

Command-based programming is not included in the basic FTC resources, but is provided by a
community project called FTCLib. SUPRA comes preinstalled with FTCLib. See sections below for more
details on installing/updating it.

For help and documentation on command-based programming and the other features offered by FTCLib,
see [the official FTCLib documentation](https://docs.ftclib.org/ftclib).

### Take a break!

From this point on, the rest of this file is more details. You should read through it, but if
you just want to jump into code, you're ready to start learning by yourself.

## Commenting

Comments are... (INCOMPLETE, PLEASE COME BACK LATER)

Comments as much for organization as they are for explaining things. If you organize your code
well, then you won't have to comment much at all. The important part is being able to retrace
your logic in case something breaks or you need to make a change.

Use `//` in a line of code to turn the rest of the line into a comment.

Use `/*` to start a block comment, then `*/` to end the block comment. All text between the start
and end of the block comment will become a comment. Block comments may span multiple lines, or
contain only a specific part of a line.

Use comments to remove unused code

Try to keep comments within the IDE line

## Best Practices

One thing, one purpose. Every method and class should have exactly one specific purpose (except
for the main loop). If one thing does multiple things, break it into multiple parts. It keeps code
self-contained and makes it easier to reuse, trace, and debug code.

The less you have to remember for something to work, the better. Either write it down or make it
obvious. The fewer unwritten rules there are, the less likely you are to make a mistake.
This is especially true if there are multiple programmers on your team.

Don't always take the shortest possible path to a solution. Think about the big picture and how
certain functions could be reused. You're laying a foundation here; if you cut corners to save
time now, you'll only lose that time later working around an incomplete solution.

Write good names. A variable named `reeklipSchmoob92` is funny, but confusing. A method named
`thisOneWorks()` will similarly make you want to install a new hole in some wall a few weeks later.
I'm looking at you, Owen.

Comment complex or vital parts of code. Don't comment everything, but you need to be able to
retrace your logic at a glance. If you can't easily tell what's going on, then what are you going
to do if it breaks? What if you make a change later that breaks it and you don't realize because
you don't remember the requirements to it working?

## Good Control Schemes

"Good control schemes are the ones where you press a button and it moves to where you want it."
-- Owen Friedman

Position control is important. That is, the motors being able to control what
position they're at and move to an exact position at the push of a button. Manually controlling
every motor is painfully slow and imprecise. Remember that from the side of the playing field, you
can't always see all parts of the robot.

The more functionality you can pack into a single button, the better. Perhaps instead of just
moving the arm to a position, pressing a button also rotates the wrist down? In the heat of the
moment mid-match, it's hard to remember to press multiple buttons at once. Just make sure your
controls don't prevent the robot from being able to do something it may need to do in a match.

If you have two drivers, try to avoid having both drivers control the same thing. It should be
very clear the duties of each driver;
i.e. have one driver control *only* the base (wheels), the other *only* controls the arm.
If both drivers can do the same thing, or one driver is doing too many things, then you might
waste precious moments in a match figuring out who does what next.

It's good to not have to remember things or read instructions mid-match. If you can make patterns
in how buttons are laid out, it will be easier to avoid pressing the wrong button.

If it suits the game, you can use a state machine. (INCOMPLETE, PLEASE COME BACK LATER)

If you use position control, you should also have a way to bypass it to control the motors manually
and reset them in case of emergency. You probably won't need it, but it would be real embarrassing
if some random technical fault caused a motor to start in the wrong spot and reduce your robot
to a pushbot just because you couldn't move it back down*.

There's endless ways to do the control scheme of your robot, but at the end of the day, it depends
on the game and the drivers. What's important is that it's both effective and intuitive.
If the drivers can play a match without looking down at the controller, that's a good sign.

*This happened to team 9987 in finals game 3 of the South Florida Regional (2023-2024).
It was a sad walk back to the pits.

## Git and GitHub

Let's straighten out a common misconception: Git and GitHub are not the same thing.
Git is the software, GitHub is the online service, and the two were created separately.

Git is what's called version control software. It tracks your code as changes are made so that
you can track changes over time and revert back to a previous working version if something goes
wrong, or if you need to look at some old code that's since been deleted.

Git works by turning your code project into a "repository" (or "repo" for short). In a Git
repository, changes are not tracked continuously, but in steps called "commits".
*Not automatically* but when you tell it to, one or more files are added to be tracked in the
*next* commit. Then, *also not automatically* but when you tell it to, a commit is made, and all
changes made to all *tracked* files since the last commit are saved in the new commit.
At any time, if you need, you can "revert" your code back to how it was in a previous commit.
Be careful, because this will *erase* all changes not in the last commit.

You can do these actions all from within Android Studio using the "Commit" and "Git" panels from
the buttons on the left side of the window. If you're courageous, then you can also try using Git's
command-line interface, but I strongly advise against this because its commands are confusing and
make it extremely easy to ruin everything.

When you make a commit, you will be asked to write a message describing the changes you made.
Do this! Write a brief but descriptive comment on what was changed in that commit. When you look
through your past commits, all you have to go off of are the messages describing each commit.
If you don't write good messages, then finding a specific change... could take a while.

Something like "fixed stuff" is not a good commit message. You'll have no clue what it means 2
weeks later.
See https://www.freecodecamp.org/news/how-to-write-better-git-commit-messages/ for some good
practices, or just google for help.

Add files and make commits when:

* You finish a new feature that works
* You finish a big change that works
* You're about to start a big change and the current code (mostly) works
* You finish fixes that you wouldn't want to redo if you have to revert
* It's been a while since your last commit

DO NOT make commits when:

* You finish a change that does not work
* You finish a change that "probably" works but isn't tested (lest you have your own mini CrowdStrike disaster)
* You are in the middle of implementing a big change and the code currently doesn't work or is confusing
* You've only made a very small change (too many commits makes it hard to find which one to look back to)
* You still have temporary variable or method names in the code (try to avoid changing existing variable/method names between commits)
* You have anything that leaves the code in a confusing, unworkable, or temporary state

Even if you work alone, you should still use Git. Being able to see or fall back on old code in a
pinch could save your life, and Git's time travel abilities make that possible without keeping
around loads of old, obsolete code. However, if your team has multiple programmers, then Git with
GitHub can also help keep your code synchronized between all of you, though the rules are a little
more complex.

GitHub is an online service that stores your Git repositories in an online system. You can then
work on a "local" copy of your "remote" online repository and when you make commits, you can
"push" them to the remote repository for your other teammates to "pull" to download and apply the
changes when they get the chance.

There's a couple new things that can go wrong when using Git online. First is that *it's nearly
impossible to safely change (amend) or delete a commit once it's pushed*. You must make a new
commit. You can cancel or amend your last commit if you forget something *before* pushing,
but not after.

Second is something called a "merge conflict". (INCOMPLETE, PLEASE COME BACK LATER)

Try to avoid the following things when working with other people:
* Working on a file that someone else is also working on (causes merge conflicts)
* Forgetting to make/push commits for an extended period of time (causes confusion and merge conflicts)
* Writing unclear variable or method names
* Changing public variable or method names which are already in use
* Spontaneously reworking public variables or methods which are already in use
* Spontaneously working on new features without checking if someone else is on it
* Leaving large chunks of important or complex code unexplained (use comments, even if just a few)
* Randomly changing the grammar of someone else's code
* Insisting on a different style of formatting your code from everyone else (being consistent is better than being good)
* Trying to undo or amend commits that have already been pushed
* Randomly deleting files that other people may be using (yes, this happens)

This is not an exhaustive list by any means.

Effective use of Git with GitHub can make for graceful collaboration between all programmers,
resulting in better code and much faster progress for a fully prepared robot. Adversely, poor
use of Git with GitHub will be a total disaster for everyone involved that takes far more time
than actually writing the code. Please, pretty please, be mindful of your changes and commits.
Follow the suggestions outlined above. Don't work on files that other team members are using.
Don't start on new features without telling anyone, and definitely don't make changes to existing
features without telling anyone.

## Libraries and Gradle

You don't write all of the code behind your project. You can use other projects, such as FTCLib
or Road Runner, in your project to get access to new features. This kind of reusable pre-packaged
code is called a library, and they can be imported into code files for access to new classes.
Some libraries come built-in to FTC robot code or SUPRA, but some need to be installed manually.
In some places, libraries are called packages. Libraries and packages usually mean the same thing.

SUPRA comes preconfigured with FTCLib and Road Runner (last updated Sep 24 2024).

Features from installed libraries will appear in your autocomplete suggestions and will be
automatically imported if used. If you want to use a new library that isn't installed, then you
will need to install it first using Gradle.

Gradle is what's called a build system. It handles downloading libraries and compiling your code
into apps so that you don't have to. If you need a new library, you should follow the instructions
provided by the library's GitHub page or website. It will probably involve adding or changing some
Gradle settings.

To change Gradle settings, first, follow the instructions given by your library to make the
appropriate edits to the settings files. They're in the Gradle Scripts dropdown in the hierarchy
on the left. The only confusing part is picking the right "build.gradle" files. There are three:
one for the project as a whole, one for "Module :FtcRobotController" (the built-in FTC code and
examples), and one for "Module :TeamCode" (your robot code). Each file is identified by the grey
text next to the "build.gradle" name. Installing a library will usually involve modifying the
files for the "Project" and/or "TeamCode" files, but not "FtcRobotController".

Finally, to apply your changes, perform a Gradle sync. You can do this either with the button at
the top right of the window, or by clicking "Sync now" on the blue banner that appears when you
make a change to Gradle settings. A Gradle sync is when Gradle reads all of the settings and
"synchronizes" your project with the settings: downloading and installing new libraries, removing
old libraries, and making sure that everything is in order. If you made a mistake in your changes,
the error will usually* pop up during the sync.
*If you forget to sync, none of your changes will apply.*

*Emphasis on the "usually". Make sure to put the right stuff in the right place. Gradle errors
often present themselves immediately, but could also appear to work at first and cause issues
later down the line.

If part of a library/package is mysteriously missing, check that it was installed correctly or
try syncing again.

Side note: Gradle settings are part of the project, but libraries are downloaded to each computer
separately. Anytime the project is opened on a new device, you have to sync and download everything
again on that device. If one person makes a change to the settings, everyone has to sync again.
Syncing requires internet if (and only if) a new library is added.

## Updating

**Installed libraries, including core FTC libraries, do not update automatically. At the start of
each season, or if you notice an out-of-date library, you must update manually.**

To update core FTC libraries, you can usually just go into 'build.dependencies.gradle' and
change the numbers. If you want, you can also go to the GitHub repository and download the new
FtcRobotController sample code files.

To update most libraries, just replace the stuff from the old installation with the stuff in the
current instructions for an installation. This might be as simple as changing a number, or it
could require tearing down and rebuilding chunks of your Gradle settings. Good luck.

If your Gradle settings ever get super borked, you can always wipe them clean by copying the
Gradle settings from the original FtcRobotController GitHub repository, then reinstalling all the
libraries. This is tedious, but sometimes it's the only option. Alternatively, if the update
isn't worth it, you can use Git to revert the Gradle settings back to the last working version.

Again, the FtcRobotController GitHub is: https://github.com/FIRST-Tech-Challenge/FtcRobotController

# FTC Robot Controller Readme

The rest of this file from this point onward is copied from the official FTC TeamCode readme.

## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""