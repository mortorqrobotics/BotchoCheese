# Solving Common (Git) Issues

It is inevitable that you are going to run into some issues with git. The goal of this file is to give you some starting points of what to try. Remember, you do **not** need to memorize all of this, and you can always look things up for further reference! 

## First Rule: Stop and Check State

Before doing **anything** when running into an issue on git, please be sure to run `git status` so you can see what is going on in terms of committed files, unstaged files, etc. You would be shocked at how many problems you can fix simply by running this command and going from there. 

## "I Staged the Wrong File!"

Sometimes, you might've run `git add` on files you didn't want to be staged yet. Say, for instance, you accidentally ran `git add secrets.txt`, so now your `git status` call looks like this: 

```bash 
Changes to be committed:
    modified: config.yaml
    modified: secrets.txt
```
To fix this, you can run `git restore --staged secrets.txt` to remove that file from the staging area. Your changes remain local, so nothing wrong with running this to avoid confusing commits! 

## "I Committed Something I Shouldn't Have!"

You can remove commits but keep your changes staged by running `git reset --soft HEAD~1`. 

If you want to also unstage those changes, run `git reset HEAD~1`. 

If you use `git reset --hard HEAD~1`, beware that you would be deleting the most recent commit on your current branch and **ALL** uncommitted changes in the staging and working directory. So try to avoid using this. 

Also, `HEAD` is referring to your current working position in the repository. It will typically point to your most recent commit.

## "I Need to Change my Last Commit!"

`git --amend` will allow you to change a commit message, add more files to the commit, etc. Only use this on your **own personal branches**. 

## "I Need to Unstage Files!"

`git restore --staged .` to unstage all files, `git restore -- staged FILE` to unstage a given file. 

## "I Need to Undo Changes to my Files!" 

If you've already committed the changes, run `git checkout HEAD -- path/to/file`. The `--` is telling you that you are specifying a file (not a commit or branch), and `checkout` is making sure that `HEAD` will include the most recent committed changes of that file. 

If the file hasn't been committed yet but is staged, you can run `git restore path/to/file` to get the same result. 

## "I Did Work on the Wrong Branch and now I Can't Find it! 

You can run `git reflog` to get an entire local history of commits. Once you run this, you can see a commit number followed by the commit message, like so: 

```bash 
a1b2c3d HEAD@{2}: commit: WIP login validation
```

That first number tells you the commit hash, which you can think of as an identifier for each commit, and you can revert back to that commit by running `git checkout a1b2c3d`. 

Note that `git log` would give you the entire history of the repository, including the remote branches, so if you are only concerned with work done locally, stick to `git reflog`. 

If you committed work to the wrong branch, you can also retrieve the work like this: 

```bash
git checkout CORRECT-BRANCH
git cherry-pick COMMIT-HASH
```

## "I'm in a Detached HEAD State!"

Sometimes, after doing that `git checkout COMMIT_NUMBER`, you will get a message like this after making more changes: 

```bash
You are in 'detached HEAD' state
```

This means that the `HEAD` pointer is no longer referring to a specific branch (like the one you were working on). It could be referring to the specific commit number, for instance. 

A detached HEAD state means that none of your changes will be saved! So, you must create a new branch and switch there: 

```bash 
git checkout -b rescue/my_work
```

Your commits will then be safe. 

## "My Branch is Different from Remote!"

If you run `git status`, we already saw in `02-core-workflow.md` that you might need to push if your branch is ahead, and pull if your branch is behind. The 3rd possible issue is that there is a divergent branch issue like this: 

```bash
git status
On branch feature/user-auth
Your branch and 'origin/feature/user-auth' have diverged,
and have 3 and 2 different commits each, respectively.
  (use "git pull" to merge the remote branch into yours)

nothing to commit, working tree clean
```

In this case, you cannot pull or push to fix this issue because you have already brought in any remote changes, and there are no more local changes to commit. You either have to merge, or you can stack commits on top of one another using `git pull --rebase`. Remember that for shared branches using merge is the safer option! 

## "How Can I Store Work for Later?"

Sometimes, you do some work on your working directory, and it is messy, so you can't commit, but you also don't want to start over. You can run `git stash` to store the uncommitted local changes elsewhere, which you can retrieve using `git stash pop`. 

## "How Can I See Differences in Local vs. Remote Branch?"

We mentioned before that you can use `git fetch` to preview files on the remote branch, but sometimes you don't want to download all of those remote files. You can run `git diff` and this will show all of the unstaged changes in the working directory. There are different commands that will show you different parts, like `git diff --staged` showing you the changes in the staging area. 

