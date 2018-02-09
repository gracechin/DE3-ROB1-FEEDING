---
title: Git Group Workflow
type: lesson
duration: "1:25"
creator:
    name: Mike Hayden, Alex Chin
    city: London
competencies: Dev Tools and Workflow
---

# Git Group Workflow

### Objectives
*After this lesson, students will be able to:*

- Understand how to use Git in a team
- Explain common git commands
- Set out a workflow for use in the team project

### Preparation
*Before this lesson, students should already be able to:*

- Understand basic Git commands
- Understand how to navigate the command-line

## Using Git in a team - Intro (10 mins)

So far in the course, you have been using Git to save your own files and submit your homeworks. However, when you leave the course, it is likely that you will need to use Git to contribute to your employer's projects and merge the fantastic code that you have written!

In order to do this effectively, we need to go through a few more git commands; commands to create branches, merge branches and resolve merge conflicts.

Up until now, you have been working directly on a `master` branch:

```bash
$ git push origin master
```

However, the `master` branch really should only be used for production-ready code. You should never really work on the master branch directly. 

#### A successful Git branching model

There are a number of different branching strategies that you can use in team work. However, this one is very efficient:

![git-model 2x](https://cloud.githubusercontent.com/assets/40461/13729141/23e02300-e926-11e5-8435-cdfed99244be.png)

## Setup your repository - Codealong (20 mins)

_Sit in your teams_

The best way to learn and understand new git commands is to practise them. So, sitting in your teams, let's create a new git repository.

### The git master

Choose one person to be the git master. 

> **Note:** In a real company, the git master would likely be a senior developer. However, during the team project, this should be the student who has the most to learn about Git. The reason for this is that they will be more involved in the codebase.

Once chosen, the git master should go to github and create a new repository.

> **Note:** Check the **Initialise this repository with a README.md** checkbox.

<img width="730" alt="screen shot 2016-03-13 at 22 59 34" src="https://cloud.githubusercontent.com/assets/40461/13732001/4f992746-e96f-11e5-9a8c-5d590c6f70f1.png">

### Contributors

Once the repository has been created you will need to add the other team members as contributors. You can do this by going to the Settings page of the Github repository. You will have to enter your Github password here.

<img width="1012" alt="screen shot 2016-03-13 at 23 01 07" src="https://cloud.githubusercontent.com/assets/40461/13732004/7dad702e-e96f-11e5-9532-06079adb98ac.png">

### Creating a `development` branch

The git master can now clone the repository with:

```bash
$ git clone <ssh-url-for-project>
```

As we don't want to work on the `master` branch, the git master should now create a development branch.

First, let's check what branches we currently have with the command:

```bash
$ git branch
```

You should see:

```bash
* master
```

We now want to create a new development branch. We can do this with the command:

```bash
$ gco -b development
```

> *NOTE:* `gco` is a `zsh` alias for `git checkout`; this was created as part of the installfest when you installed **oh-my-zsh**.

You should see `Switched to a new branch 'development'`. This command `gco -b` is short for:

```bash
$ git checkout -b development
```

This will allow you to create a new branch and switch to it directly. We could do this in two steps, e.g.

```bash
$ git branch development
$ git checkout development
```

If we now check the branches again using:

```bash
$ git branch
```

You should see:

```bash
* development
master
```

#### Local vs remote branches

Currently, this branch is only on the git master's computer. It is a local branch. We want this branch to be accessible by all of the other team members, so we need to push it to Github and make it a remote branch.

We can do this with:

```bash
$ git push origin development
```

Great, if we have a look on Github now, we should see that there is a remote branch called `development`.

<img width="326" alt="screen shot 2016-03-13 at 23 03 59" src="https://cloud.githubusercontent.com/assets/40461/13732016/ea77d1ea-e96f-11e5-8f5d-bea4756b344f.png">

### Clone the repository

Now, everyone in the team can clone the repository. However, a normal clone of a repository will only checkout the master branch. To checkout the remote development branch, you need to do:

```bash
$ gco -b development origin/development
```

You should see `Branch development set up to track remote branch development from origin.`. If you run the command `git branch`, you should see:

```bash
* development
master
```

You should see also what branch you are on in the command-line prompt.

> **Don't fork?!** Why aren't we forking the repository? Whilst you might use a forked repository workflow for a larger project, the process of creating and resolving pull-requests would be too slow for this small team.

## Contributing - Codealong (10 mins)

Now, one of the team members should make a change to the codebase. Whilst you "could" make a change on the development branch, it is better practice creating a new feature branch for this change.

```bash
$ gco -b update-readme-file
```

Now, open the `README.md` file in your text editor and add your name directly beneath the title.

Once you have done this, you can add and commit this change with a relevant message:

```bash
$ git add .
$ git commit -m "Added name to README"
```

Now, the process that you want to do in order to merge your changes to `development` is:

First, pull the most recent changes from the remote development branch onto the local development branch:

```bash
$ gco development
$ git pull origin development
```

We know that there are no changes now but when you are working in a larger team there may well be some changes.

Now, checkout your feature branch and merge development into that feature branch:

```bash
$ gco update-readme-file
$ git merge development
```

> **Note:** We're merging `development` into the feature branch (`update-readme-file`) instead of the other way around because if something breaks, then you haven't broken the `development` branch.

In this case, we have no conflicts, so now we can merge the feature into the `development` branch.

```bash
$ gco development
$ git merge update-readme-file
```

Once we have done this, we can push this change to the remote `development` branch (on `origin`).

```bash
$ git push origin development
```

Great!

## Creating a merge conflict - Codealong (10 mins)

At this point, the code on both the git master's local machine and the other team members machines are "behind" the remote `development` branch.

Now, another team member should create a branch to edit their Readme.

```bash
$ gco -b update-readme-file
```

Next, the student should try to add their name on the same line of the `README.md` file. It **must** be the same line as we are deliberately trying to create a conflict. Git is pretty clever so it needs to be exactly the same line.

Once you have done this `add` and `commit` the change.

```bash
$ git add .
$ git commit -m "Adding name to readme"
```

Now go through the same process to pull the most recent code onto development and then merge the feature branch:

```bash
$ gco development
$ git pull origin development
$ gco update-readme-file
$ git merge development
```

You should now get a merge conflict!

#### Conflicts are NOT scary.

When starting out to use Git, merge conflicts seem confusing and scary. 

However, they are not. Git is just marking a file to say that two changes have occurred, and it needs some help to choose which one to pick.

#### Resolve the conflict

Remove the conflict messages and keep both names.

You now need to add the new changes.

```bash
$ git add .
$ git commit -m "Resolving the merge conflict in Readme"
```

Now, you can merge the code onto `development` and push to the remote branch:

```bash
$ gco development
$ git merge update-readme-file
$ git push origin development
```

Great!

## Independent Practice - Codealong (10mins)

Now, work together to repeat the previous steps with the git master. Don't pull the `origin development` branch so that you can create a merge conflict.

## Merge to master - Codealong (10mins)

Once the code has all been merged AND tested! The git master can then merge `development` onto the `master` branch and push to `origin master`. This should ONLY be done when the code is ready to go live.

```bash
$ gco development
$ git merge master
$ gco master
$ git merge development
$ git push origin master
```

## The Rules of the Road - (10 mins)

1. Never work directly on the `master` branch.
2. Never work directly on the `development` branch.
3. Make a feature branch for each card in Trello.
4. Create feature branches from the `development` branch.
5. Name the branch after the Trello card.
6. Once you've completed your feature, merge the `development` branch into your feature branch.
7. Git master to merge completed branches into `development` branch.
8. Git master to merge `development` into `master`.
9. __NEVER__ push broken code to `master`.
10. use the `--no-ff` flag when merging feature branches in to the `development` branch.

## Some useful commands

| Command | Action |
|---------|--------|
|`git checkout -b <branch-name>`| Create a new branch from the branch you are currently in, and switch to that new branch. |
|`git push origin <branch-name>`| Push the newly created branch to github. |
|`git checkout <branch-name>`| Move to (check out) a branch. |
|`git pull origin <branch-name>`| Pull the branch from origin and __merge it into the current branch.__ |
|`git reset --soft <commit-hash>`| Move commited changes to staging. |
|`git reset --hard <commit-hash>`| Roll back to the commit. __WARNING: This is a distructive action!__ |
|`git status`| Check the status of the current branch. |
|`git log`| Check the previous commits. |
|`git reflog`| Check previous actions (including movements between branches. |
|`git merge <branch-name>`| Merge `<branch-name>` into the current branch. |
|`git merge <branch-name> --no-ff`| Bundle the branch into one commit when merging. __NB: This is helpful when merging feature bracnhes into the development branch.__ |

## Other useful tips

1. Seek advice when doing your initial branches and merges.
2. Communicate with each other.
3. Don't point fingers or blame team members for their mistakes.
4. Remain calm at all times.
5. If things go wrong, get help from an instructor.

## Further reading

- [A successful Git branching model](http://nvie.com/posts/a-successful-git-branching-model/)
- [Git Branching - Branching Workflows](https://git-scm.com/book/en/v2/Git-Branching-Branching-Workflows)