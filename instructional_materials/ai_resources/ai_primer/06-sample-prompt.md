# Sample Prompt for Better Answers 

To put everything together, we will walk through a quick problem where you can implement all of these tips. 

## Case Study: Data Quality Assurance 

You have a .csv file (think Excel/Google Sheets) and you need to validate that the rows are proper. 

You decide to use GenAI to speed up this process.

### Turn 1: Good Context + Constraints 
Use the following prompt: 

```text 
You are a senior Python engineer mentoring a beginner.

I am working on a function that parses a CSV row representing a user.

Each row has this format:
username,age,email

Rules:
- username: non-empty, no spaces
- age: integer between 13 and 120 (inclusive)
- email: must contain exactly one "@" and at least one "." after "@"

The function should:
- Return a dictionary with keys: username, age, email
- Raise a ValueError with a clear message if validation fails

Please:
1. Restate the requirements in your own words
2. Outline a step-by-step approach (no code yet)
```

Why is this a good prompt?
- Role prompting to narrow scope of AI answers 
- Explicit context that gives model clear idea of the problem 
- Constraint-based prompting to structure your output in a format you can use 
- Decomposition by encouraging a step-by-step approach so you can clearly see its thought process

### Turn 2: AI Response 
We can assume the AI response:
- Restates rules 
- Lists validation steps
- Mentions parsing, checking fields, raising errors. 

If by any chance it doesn't (which is rare, but remember these outputs have randomness in them!), you can always re-prompt the AI with something simple like this: 

```text
You mentioned that rule #2 looked like this: <wrong text>

However, change the rule so it reflects the original rule: <correct text>

```

Then, you should be on your way. You can apply similar logic to any follow-up steps - give the AI context and be direct with what you want changed! 

### Turn 3: Consistency Lock + Feedback 

Start off with this prompt:

```text
Looks good.

One clarification:
- Usernames like "john_doe" are allowed
- Usernames like "john doe" are not

Update your requirements summary to reflect this.
Do not change the approach yet.
```

Why this is a useful follow-up:
- Validate that most of the output is correct so AI has some direction (and flattering it works to make it more receptive... seriously look it up)
- Force an explicit update with the exemplar so it has a clear idea of what to change 
- Prevent silent assumption drift by telling it to not alter the fundamental approach 

### Turn 4: Approach Refinement

Once that is fixed, we can follow up with the actual approach change: 

```text
Now revise the step-by-step approach to reflect the clarified username rule.

Keep it high-level.
```

Again, we reinforce the approach, while asking only to revise with the username. 

### Turn 5: Controlled Implementation Prompt

Now we are ready to have it give us some code in Python: 

```text
Now implement the function in Python.

Constraints:
- Use only standard library features
- Keep the function under 25 lines
- Use clear variable names
- Do not add extra features

After the code:
- Briefly explain how each validation rule is enforced
```

We have tight constraints on our code. We also have some elements of Chain of Thought (CoT) because we are asking about how the validation rules are followed. 

### Turn 6: Verification via Rule Restatement
At this point, we don't want to proceed without confirming that our model is following our rules. 

Let's use this prompt to check that: 
```text
Before we go further:

1. Re-list all validation rules
2. For each rule, point to the exact part of the code that enforces it
```

This might seem a little repetitive, but AI's sometimes struggle with consistency in response - they might claim they implemented a rule (and maybe they did), but they could "forget" that rule in later responses. Here we check that the model understands why it is coding what it is coding. 

If some rules are missing, simply ask for a rewrite using the given rules. 

### Turn 7: Edge Cases
With almost every function, you will be dealing with some edge cases, particularly with a complex field with many moving parts like in FRC. 

We ask for that with this prompt: 
```text
Now test the function conceptually with these cases:

1. "alice,13,alice@example.com"
2. "bob,12,bob@example.com"
3. "charlie,25,charlieexample.com"
4. "   ,30,test@example.com"
5. "dana,120,dana@test.co.uk"
6. ""   (empty string)

For each case:
- Should it pass or fail?
- Which rule applies?
- What error message should be raised?

Do not change the code yet.
```

Why does this do a proper edge case check?:
- Few shot prompting via the exemplars to demonstrate what an edge case looks like 
- We also make sure we get some feedback on why those edge cases should pass or fail (empty values, missing separators, etc.)

### Turn 8: Self Review 
You can be fairly sure that the first "final" answer the model gives you will not be perfect - it may be *really* good, but it might not be exactly what you want. This will hold true especially as you produce more complex code. 

One good way to find out what could be improved is with a prompt like this: 

```text
Now review the final solution and answer:

1. One potential bug or limitation
2. One improvement you would suggest for a production system
3. One thing a beginner should remember from this example
```

Here, we check for any issues, possible improvements, and a basic summary that shows learning outcomes. 

You have now seen how prompting can give you much improved outputs - happy prompting and programming! 