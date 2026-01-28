# Context is Key

## Why Context Matters 

Context is the information you give the model to get an answer. You give the model context in the form of prompts, which are simply the text you give the model in the form of a question (usually). 

Since models cannot read your mind (at least not yet, anyway), the answer you get from them will only be as good as the context you give them. 

For example, say we create a function to calculate someone's age based on their birthday. However, we haven't taken into account the day/month, just the year, so some ages might be wrong.

Below is a poor way to prompt (or ask) the AI to fix this issue: 

```text
Fix my code:

def calculate_age(birth_date):
    today = datetime.date.today()
    age = today.year - birth_date.year
    return age
```
The model has no idea what to fix here! 

Instead, you should point out the error in the function, and be a little more specific in what exactly you want the model to do: 

```text
I've built out the below function to calculate age given date of birth. It currently only takes into account year, but it needs to account for month and day. My function is below: 

def calculate_age(birth_date):
    """
    Calculates age based on birth_date (datetime.date object).
    """
    today = datetime.date.today()
    # Potential Bug: This doesn't account for the current month/day
    age = today.year - birth_date.year
    return age

```

Now, the model knows what our function is supposed to do, what it currently does, and what it needs to do to close that gap. 

## Limits to AI Usage 

Unfortunately, we do not have infinite prompts to use to make sure we get the model to do what we want it to do. AI companies have to make money somehow! 

### Tokens 

Every character that you type into the prompt takes up tokens, which are units of text that the AI processes. Additionally, anything that the AI outputs takes up tokens as well! A general rule of thumb is that 100 words is roughly 75 tokens, but this does vary a bit from model to model. 

### Context Windows 

The maximum number of tokens in a given conversation with a model is known as the context window. In general, LLMs are "stateless" in that they do not remember past prompts. However, if you haven't used up the entire context window, the LLM will remember your past questions and use that as context to answer your newer questions. So, it is **crucial** that you conserve tokens. 

### How to Conserve Tokens 
There are a few ways you can avoid running through your context window too quickly: 
- Be concise; get to the point, don't write an essay where it isn't needed 
- Keep conversations on a single topic - if you need help with an unrelated problem, start a new conversation 
- Request efficient formats (literally ask the model for structured outputs like JSON, for example)

Ultimately, your context window depends on your model. ChatGPT-4o mini, the free tier, has a 16K token context window. This might seem like a lot, but when generating longer code, this goes quickly. 