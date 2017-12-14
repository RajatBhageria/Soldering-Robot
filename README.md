# Soldering-Robot-
Robot automatically does trajectory planning, feeds wire in front of rod, and solders to a pin on a perfboard 

To test, simply run testRobot and pass in two arrays, a nx1 matrix of pinLetters on a perfboard and a nx1 matrix of pinNumbers on the perfboard. 

An example might be pinLetters = ['A';'B';'Z';'C'] and pinNumbers = [1;2;5;20]