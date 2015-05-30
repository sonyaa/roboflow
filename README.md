# RoboFlow
This is the current version of RoboFlow (see ICRA 2015 paper: "RoboFlow: A Flow-Based Visual Programming Language for Mobile Manipulation Tasks"). It works for ros hydro and has a web-based visual programming language editor. However, it currently only works for the top-down (starting from scratch) approach, rather than bottom-up (starting with a demonstration). 

Don't hesitate to shoot me an email (sonyaa at cs.washington.edu) with any questions!

This version requires the following packages to be also installed (they're ros hydro catkin packages):

    https://github.com/sonyaa/pr2_pbd_app (web-based app for saving manipulation demonstration actions + backend for executing those actions)
    https://github.com/hcrlab/rws_pr2_navigation (web-based app for saving navigation locations + backend for actually navigating there)

For the previous all-in-one ros-groovy version, see https://github.com/sonyaa/pr2_pbd

# Installation
This project is built using Gulp (build system) and Polymer (web component framework). In order to build the project, we first have to install Gulp and its dependencies. First, install (Node.Js)[https://nodejs.org/], (Ruby 2.1.*)[https://www.ruby-lang.org/en/downloads/], then (Sass)[http://sass-lang.com/install].

Next, clone this project and cd in the project's top level directory. Run the following commands to install some node modules we will be using:

    npm install -g bower
    npm install -g yo

Finally, we can use npm and bower to install the remainder of the dependencies which are specified in the package.json and bower.json files:

    npm install
    bower install

If you run into any conflicts between bower packages, always choose the package with the highest version number. Additionally, ensure that you are using the Sass 2.1.* version and not the 2.2.* version as this may cause build errors.

# Launching the web GUI
Once all of the dependencies are installed, you can build the project by running gulp from the top-level directory of the project. If you want to build and view the application (serve it to your browser), run gulp serve.

# Launching on the robot
On the robot, launch the following:

    roslaunch roboflow app.launch
    roslaunch rosbridge_server rosbridge_websocket.launch
