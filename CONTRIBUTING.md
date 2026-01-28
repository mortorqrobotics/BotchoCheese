# Git (Version Control) Standards

## What are these Standards?

These Standards are rules for us to follow when contributing to our code base. These should be followed at all times! 

## Why do we need these Standards?

We want to have consistency in our code contributions. Everyone should be able to easily see what changes have been made, why, and who made them. </br>

These standards will help us do this. These can change based on what the team deems appropriate, but the general ideas will likely remain the same.

## Branching Strategy 

### Branch Name Conventions 

We want everybody to contribute code. In order to keep this organized, you will be doing all code on a personal branch, and will put your code on the main branch through a pull request (see "Pull Requests" below).

Some general naming conventions are: 
1. Only lowercase letters
2. Keep names short and to the point - nobody should have to ask what a branch is doing, but don't write a whole sentence 
3. Hyphens (-) instead of spaces
4. No special characters (!, $, etc.)

To categorize our branches, we use the below groupings, with NAME at the end to indicate who is creating the branch: 

`feat/feat-name/NAME`: Creating a new feature (e.g., `feat/dual-cam-vision/Nick`) </br>
`fix/bug-name/NAME`: Debugging fix (e.g., `fix/shot-trajectory/Nick`) </br>
`test/test-name/NAME`: Anything you want to experiment around with (e.g., `test/rear-cam-vision/Nick`) </br>
`other/other-name/NAME`: Anything that does not fit in any of the above categorizations (unlikely you'll use this but still helpful to have) 

## Commit Conventions

### Committing Files 

We want commits to be "logical units". This means that when you create a new feature, for example, you should commit **all files** that you changed as a part of that feature construction to keep the history of our changes more clear. 

### Commit Messages Conventions 

Commit messages should be concise and informative. A typical message style might look like this: 

```
<type>(<scope>): <subject>

<body>

<footers>
```

`<type>`: describes the nature of the commit
   - `feat` for a new feature
   - `fix` for bug fixes
   - `test` for code tests
   - `style` for grammatical corrections
   - `refactor` for changing code style to be more readable (not a functional change)
   - `other` for anything not encompassed in the above categories (again fairly rare but useful to have this option)

`<scope>`: optional description of the part of the codebase (e.g., feat(cam_vision)) </br>
`<subject>`: concise summary of the commit - shouldn't surpass 50 characters, and should omit longer descriptions that belong in the body. </br>
`<body>`: optional detailed description of the commit - should probably not exceed 72 lines, and would only save this if you are making a large change to how the codebase works </br> 
`<footers>`: optional metadata (e.g., what pull request a commit addresses)

Here is a sample commit message that follows all of the above guidance:

```
refactor(rear_camera): reorganized target detection function

Refactored target detection function by turning main coding body into 3 helper functions.

Addressed PR #07 regarding less verbose code

``` 

Note that to do multi-line commit messages in terminal, you can structure it as: 

```
git commit \
-m "refactor(rear_camera): reorganized target detection function" \
-m ...
```


### Personal Branch Commit Frequency 

Since nobody else is touching this branch but you, you can frankly commit whenever you want. Some general rules of thumb are: 

1. If you are leaving your work for an extended period of time (e.g., longer than a bathroom break), commit your code.
2. If you are stumped and find yourself needing to do more research or take some time to brainstorm, commit your code.
3. If you've completed an important step in your code (e.g., debugged an important function), commit your code.

Ultimately, someone else reading your commit history should be able to understand what is going on without asking you. 

## Pull Requests

### Pull Request Checklist 

Since this code will be changing our codebase, the rules here are going to be a little more strict. For your code to be pushed to the main branch and therefore override the existing code, you will have to create a pull request (PR) so that your changes can be approved. Below are some things that **must be done** before your PR is created. 

1. Do not commit any code that will break anything! So, you should verify that your code works **in your own branch**.
2. Only commit when you've completed important steps. Debug a function, write a new class, etc.

### Review and Label Conventions 

For any of your PRs, add Elle, Marmo, and Nick so we may review. Elle and Marmo will check your work first, and will scale up to Nick if necessary. </br>

Add a label to your PR. GitHub provides some labels to mark our PRs for better organization. The list can be found on the PR page (under Labels).




