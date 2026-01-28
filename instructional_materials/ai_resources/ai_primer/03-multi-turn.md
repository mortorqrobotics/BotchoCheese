# Multi-Turn Conversation: Asking More than One Question 

## What is Multi-Turn? 

A turn in an LLM conversation refers to a single sequence of an input (your prompt) followed by an output (the model response). A multi-turn conversation is more than one of these sequences. 

## Why does this difference matter? 

In a single turn conversation, the only context the LLM is handling is your original prompt, and its original output. However, if you ask follow-up questions, its context now includes those questions and corresponding outputs. LLMs have the tendency to "get lost" and "forget" the previous sequences when generating their new outputs. 

However, strong prompting can help mitigate this problem and allow you to get more out of the longer conversations. 

## Prompting for Multi-Turn 

To illustrate how multi-turn can work, let us run through an example. In this example, we are trying to build out a function that detects if a password is valid based on a set of rules. 

### Turn 1: Initial Prompt 
Here, we can start pretty broad in terms of our ask:

```text
I need a Python function that checks whether a password is valid.

Rules:
- At least 8 characters
- Contains at least one number
- Contains at least one uppercase letter

Write a simple solution.
```

The model will almost certainly output a function that handles most passwords correctly, if not all of them. However, we want to treat this 

### Turn 2: Consistency Lock-in Prompt
We want to make sure that the model is acknowledging the rules we set and putting those into the solution. We can prompt this as:

```text
Before changing anything, summarize the rules your function is enforcing in a numbered list. 
```

This will help us ensure consistency in responses. 

### Turn 3: Feedback 
Suppose we notice that the function allows spaces as a valid character in a password, which we don't want. We can give it this prompt to fix that: 

```text
Good. One issue: your function allows spaces, which I want to disallow.

Do NOT rewrite the full function yet.
Explain where the current logic fails and what needs to change conceptually.
```

Notice that in this step we want to see some reasoning first before we let the model run wild with rewriting some of the code. 

This form of feedback is also known as **Human-in-the-Loop**. 

### Turn 4: Targeted Modification 
Now, we want to see the code change: 

```text
Now update only the part of the function needed to enforce:
- No spaces allowed

Keep everything else exactly the same.
```

Here, we specify the exact change we want, and emphasize that we don't want to see any other code changed. 

### Turn 5: Consistency Verification
We want to verify that our modification followed our rules, which we can check by doing this: 

```text
Re-list all validation rules again.
Then confirm that the updated function enforces each one.
```

### Turn 6: Edge Cases 
Assuming our consistency verification passes, we can move on to ensuring our code handles edge cases, or situations that are more rare but require special attention because they tend to be at the boundaries of what our code was meant to handle. We can stress test our code with a sample prompt like this: 

```text
Now test the function with the following:

1. A normal valid password
2. A password that fails exactly one rule
3. A password that fails multiple rules
4. A true edge case that could easily be overlooked
   (for example: an empty string, a string of only spaces,
   or a string exactly 8 characters long)

For each case:
- State whether it should pass or fail
- Explain which rule(s) apply
```

Now, we will have a robust function that solves our use case! 
