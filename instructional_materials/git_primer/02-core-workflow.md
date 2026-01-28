# The Core Git Workflow

This is basic workflow you will use the vast majority of the time you are using git. 

## 1. Check status
```bash
git status
```

Always start here. To explain what outputs you might see when running this, let's look at an example output: 

```bash
On branch feature/user-auth 
Your branch is ahead of 'origin/feature/user-auth' by 1 commit. 
  (use "git push" to publish your local commits)

Your branch is behind 'origin/feature/user-auth' by 2 commits.
  (use "git pull" to update your local branch)

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
        modified:   src/auth/login.py
        new file:   src/auth/session.py
        deleted:    src/auth/legacy_auth.py

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
        modified:   src/auth/login.py
        modified:   README.md

Untracked files:
  (use "git add <file>..." to include in what will be committed)
        notes/debugging.txt
        tmp/test_output.json

```
Let's go through each of these lines to see what this means. 

```bash
On branch feature/user-auth 
```

This tells you what branch you are doing your work on in the working directory. This can also be referred to as programming "locally". 

```bash
Your branch is ahead of 'origin/feature/user-auth' by 1 commit. 
  (use "git push" to publish your local commits)
```

The `origin/feature/user-auth` is the repository branch that corresponds to our working directory branch; we know it is remote because it has `origin` at the beginning. 

Our branch being ahead by one commit means that we have made changes, used `git add` to stage them (meaning they are ready to be pushed to our remote repository), but have not actually used `git push` to get them there. 

```bash
Your branch is behind 'origin/feature/user-auth' by 2 commits.
  (use "git pull" to update your local branch)
```

So how is this command also possible? How can we be ahead of our remote branch by one but also behind by 2? This is because earlier, there were two new commits pushed to our remote repo, but we never used `git pull` to get those changes to our local branch. We will address how to address this issue of getting remote changes into our local branch a bit later, but for now know that usually you can just run `git pull` and those changes will be reflected in your local branch. 

```bash 
Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
        modified:   src/auth/login.py
        new file:   src/auth/session.py
        deleted:    src/auth/legacy_auth.py
```

These are changes that are in the staging area, and only these changes will be able to be committed and pushed to our remote branch. Note that it makes the distinction between files that are modified, added, or deleted. 

```bash 
Untracked files:
  (use "git add <file>..." to include in what will be committed)
        notes/debugging.txt
        tmp/test_output.json
```

If a file is untracked, this means it has been changed, but is only in the working directory and not in the staging area. So, no changes here will be committed. Of note, `login.py` is included in **both** the staging area and the working directory, which implies that we made changes, put them in the staging area, but then made more changes and did not run `git add` to get it into the staging area. 

## 2. Make Changes 

At this point, you should feel comfortable changing your files as is. Once you do that, move onto the next step. 

## 3. Stage Changes

You can stage changes for an individual file like so: 

```bash 
git add FILE
```

where `FILE` is replaced by the file you want to stage the changes for. You can also add multiple files by listing them out like `git add FILE1 FILE2` and so on.  

Note you can add all changes in the working directory by replacing `FILE` w/ `.`, but it is usually easier to list out individual files for keeping commits as logical units instead of just every change you've made. 

## 4. Commit Changes 

You can do a commit like so: 

```bash 
git commit -m "Here is my commit message!
```

A commit message is a helpful (and optional) way to explain what your commits are for. We cover how to do your commit messages in the `CONTRIBUTING.md` file, so for now just understand that you should always add a message so people understand what you are doing! 

## 5. Sync w/ Remote

We discussed this earlier, so here is the way you should handle commits: 

```bash
git pull
git push
```

`git pull` ensures all remote changes are reflected in our local branch, `git push` so that our local changes are reflected in the remote branch. It is **crucial** to run things in this order so your commit history is an accurate reflection of the programming changes. 