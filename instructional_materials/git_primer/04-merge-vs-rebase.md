# Merge vs. Rebase: Different Ways of Resolving Remote and Local Changes

OK, so you've made some changes locally. You're finally ready to get those changes on the `main` branch - how do you go about doing this?  

## Merge: Combining Branch History 

With `git merge`, you are essentially combining the commit history of two branches. So, you aren't erasing any of the previous work; you are simply making sure that the `main` branch has access to your own branch's commits. 

The syntax is straightforward. You first run `git checkout main` to get back to your target branch (or replace `main` with any other target branch you want to merge with): 

```bash
git merge BRANCH_TO_MERGE
```

where the `BRANCH_TO_MERGE` is the branch you were doing work on that you want to merge with the target branch. 

## Rebase: Rewriting Commit History 

Sometimes, you realize that you don't want to run merge, because you need to change the commit history. 

For example, say someone committed some change to the repository while you were doing some work locally. You realize that the work you did should come before their work. So, you need to change the commit history to reflect that. 

There are several ways to run `git rebase` to change the commit history. However, these will be omitted because rebase is fairly challenging to work with when you have many people working on the same repository. Feel free to do some research on useful `rebase` commands, but you likely will be better off using `merge`. 

## Fetch: Previewing Remote Content

You might only be interested in viewing the remote content without changing your local branches. Running `git fetch` allows you to download remote content to your working directory and you can therefore view any changes manually. 

You likely would be looking at files on the repo website anyway, but this is still useful if you'd rather have files stored locally as well. 