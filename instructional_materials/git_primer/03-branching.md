# Branching: So you don't break things! 

Branches exist so you can work **without breaking main**.

We cover branch naming conventions in our `CONTRIBUTING.md` document. 

In the `git-mental-model` file, we talked about how branches are pointers for commits; they don't change anything on `main` **unless** you merge the changes. 

So, working on branches makes your life easy because you can do whatever you want without worrying about messing up the original code. 

## Branching syntax

Creating a branch: 
```bash
git -b BRANCH
```
where `BRANCH` is your branch name. 

Switching to a branch: 
```bash
git checkout BRANCH
```

Once you switch to a branch, this means any work you are doing on a file will be reflected **only in that branch**. 

You can also combine commands. 

Creating **and** switching to a branch: 

```bash 
git checkout -b BRANCH
```

Once you are on your own branch, you can get to programming! 