# Prompt Engineering: More Advanced Techniques to Get Better Prompts (and Better Answers)

## What is Prompt Engineering? 

Prompt engineering is essentially what we've been doing up to this point; we have been optimizing the inputs we give our models for achieving the desired outputs. 

In this file, we're going to explore some more advanced topics in prompt engineering that may help you solve more complex problems. 

There are a **ton** of techniques for prompt engineering, so don't be afraid to look up some more ideas. These will give you a great place to start. 

## Chain-of-Thought (CoT) Prompting 

Chain-of-Thought (CoT) prompting refers to the process of getting an LLM to reveal its intermediate steps in reaching a decision, which will give you more insight into how an answer is generated and therefore can direct you towards how to improve that output. 

Generally, this is achieved in two different ways: simple cue, and exemplars. Simple cue means providing instructions in the prompt like `Let's think step-by-step`. Exemplars are sample questions and answers that are meant to highlight the type of response that the LLM could be. A prompt that uses both could look like this: 

```text
You are helping a beginner programmer.

Here is an example of how to reason about a problem:

Example:
Question:
Why does this code print 0? Show your reasoning. 

x = 5
if x > 10:
    print(x)
else:
    print(0)

Reasoning:
1. The variable x is set to 5.
2. The condition checks whether x is greater than 10.
3. Since 5 is not greater than 10, the condition is false.
4. The else branch runs and prints 0.

Final Answer:
It prints 0 because x is not greater than 10, so the else block executes.

---

Now apply the same reasoning style to this problem:

Question:
Why does this function return None? Think step by step

def add(a, b):
    a + b
```

The `Show your Reasoning` and `Think step by step` lines are examples of simple cues, and the sample question is a good exemplar. 

Note that using multiple exemplars is also known as few-shot prompting, and using no exemplars is zero-shot prompting. 

## Role and Persona Prompting 

Most LLMs are not trained to solve very specific tasks - they can generally generate code, for example, but are not specialized in generating code for robotics. This is where directing how the model should act can give you better outputs.

By giving the model a certain role/persona, you are implicitly telling the model assumptions about knowledge, level of detail needed, etc. This will give you much more appropriate outputs for your use case.

For example, if you are trying to have the LLM give you a more robotics-level response, you could try something like this: 

```text
You are a senior robotics engineer specializing in teaching a high school robotics team software development. Explain this bug without using jargon:
```

This puts the LLM in more of a "teaching" role and will give you a more digestible explanation. 

## Constraint-Based Prompting 

Sometimes we want the model to avoid doing certain things. By explicitly mentioning the constraints it is operating under, you will ensure you get appropriate outputs. For example: 

```text
I need you to build a function that calculates the next number in a fibonacci sequence. Below are some rules: 
- Do not use recursion 
- Use only native Python 
```

## Output Control 

A lot of your output might need to be code that can be copy and pasted. You might also want a way to avoid having the LLM ramble on, since they do tend to prefer more verbose answers. You can force a structured output like so: 

```text
Respond using this format: 

Explanation:
- ...

Code: in Python, 15 lines
```

## Decomposition Prompting

Breaking down the problem into steps can give the model a framework of how to answer your question instead of spewing out a long, drawn-out answer. For instance: 

```text
I need some code that counts the number of odd numbers in a sequence. Follow these steps: 
1. Describe the algorithm in words
2. Write pseudocode
3. Write code in Python 
```

