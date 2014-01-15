2014 Robot Code
===============

Hello Everyone!<br>
Welcome to FRC 2014 with Fairview High School!<br>
This will be our working repository for the season.

An AWESOME (free) tutorial for git can be found [here](http://www.codeschool.com/courses/try-git)! Learn it!

Repository Rules:<br>
1) The master branch is **protected**! This means you can not commit to master.<br>
	(See Procedure #2 on how to get your code approved into master.)<br>
	The reason for this is so that we can keep the master branch working at all times.

2) Work on the development branch or feature branch. Try to follow the rules outlined [here](http://nvie.com/posts/a-successful-git-branching-model/).

Procedure:
0.5) Check out the repo with `git clone https://github.com/fairviewrobotics/2014_Robot_Code.git`

1) Starting a new feature branch
	1.A) run the command `git checkout -b myfeature develop`

	This makes a new branch off the develop branch, here you can work on your new feature without fear of merge conflicts! (ewww)
	
2) Finishing a feature branch (merging it into develop)
	2.A) `git checkout develop`
	2.B) `git merge --no-ff myfeature`
	2.C) OPTIONAL `git branch -d myfeature`
	2.D) `git push origin develop`

3) Getting Code Approved To Master Branch
	3.A) In the right bar of the repo, click the "Pull Requests" button. (Looks like three points and an arrow)
	3.B) Click the "New Pull Request" button.
	3.C) In the "base" section, select the master branch (you want to merge your branch into master).
	3.D) In the "compare" section select the branch you want to have merged into master.
	3.E) Click the text box to create a pull request.
	3.F) Under "Title" type an short description of what your code changes
	3.G) Under "Description" add a comment describing
	3.H) Select "Send Pull Request"

DO NOT merge your branch into master on your computer, it makes the master branch unsafe and should be avoided.
