2014 Robot Code
===============

Hello Everyone!<br>
Welcome to FRC 2014 with Fairview High School!<br>
This will be our working repository for the season.

An AWESOME (free) tutorial for git can be found [here](http://www.codeschool.com/courses/try-git)! Learn it!

## Repository Rules:

1. The master branch is **protected**! This means you should not commit to master. (See Procedure #2 on how to get your code approved into master.) The reason for this is so that we can keep the master branch in a working state at all times.
2. Work on the development branch or feature branch. Try to follow the rules outlined [here](http://nvie.com/posts/a-successful-git-branching-model/).

## Procedure:

1. Check out the repo with `git clone https://github.com/fairviewrobotics/2014_Robot_Code.git`
2. Starting a new feature branch
	1. run the command `git checkout -b myfeature develop` - This makes a new branch off the develop branch, here you can work on your new feature without fear of merge conflicts! (ewww)
3. Finishing a feature branch (merging it into develop)
	1. `git checkout develop`
	2. `git merge --no-ff myfeature`
	3. OPTIONAL `git branch -d myfeature`
	4. `git push origin develop`
4. Getting Code Approved To Master Branch
	1. In the right bar of the repo, click the "Pull Requests" button. (Looks like three points and an arrow)
	2. Click the "New Pull Request" button.
	3. In the "base" section, select the master branch (you want to merge your branch into master).
	4. In the "compare" section select the branch you want to have merged into master.
	5. Click the text box to create a pull request.
	6. Under "Title" type an short description of what your code changes
	7. Under "Description" add a comment describing
	8. Select "Send Pull Request"

DO NOT merge your branch into master on your computer, it makes the master branch unsafe and should be avoided.
