# Contributing to RoSys

We're thrilled that you're interested in contributing to RoSys!
Here are some guidelines that will help you get started.

## Reporting issues

If you encounter a bug or other issue with RoSys, the best way to report it is by opening a new issue on our [GitHub repository](https://github.com/zauberzeug/rosys).
When creating the issue, please provide a clear and concise description of the problem, including any relevant error messages and code snippets.
If possible, include steps to reproduce the issue.

## Contributing code

We are excited that you want to contribute code to RoSys.
We're always looking for bug fixes, performance improvements, and new features.

## Setup

To set up a local development environment for RoSys, you'll need to have Python 3.10+, [poetry](https://python-poetry.org/) and pip installed.

You can then use the following command to install RoSys in editable mode including the development dependencies:

```bash
poetry install --with dev
```

This will install the `rosys` package and all its dependencies in a poetry development environment.

## Coding Style Guide

### Formatting

We use [pre-commit](https://github.com/pre-commit/pre-commit) to make sure the coding style is enforced.
It will be installed with the setup step above, but you then need to install the corresponding git commit hooks by running the following command:

```bash
pre-commit install
```

After that you can make sure your code satisfies the coding style by running the following command:

```bash
pre-commit run --all-files
```

These checks will also run automatically before every commit.
The command may fail with

> RuntimeError: failed to find interpreter for Builtin discover of python_spec='python3.10'

You will need to install Python 3.10 and make sure it is available in your `PATH`.

### Formatting

We use [autopep8](https://github.com/hhatto/autopep8) with a 120 character line length to format our code.
Before submitting a pull request, please run

```bash
autopep8 --max-line-length=120 --in-place --recursive .
```

on your code to ensure that it meets our formatting guidelines.
Alternatively you can use VSCode, open the rosys.code-workspace file and install the recommended extensions.
Then the formatting rules are applied whenever you save a file.

In our point of view, the Black formatter is sometimes a bit too strict.
There are cases where one or the other arrangement of, e.g., function arguments is more readable than the other.
Then we like the flexibility to either put all arguments on separate lines or only put the lengthy event handler
on a second line and leave the other arguments as they are.

### Imports

We use [ruff](https://docs.astral.sh/ruff/) to automatically sort imports:

```bash
ruff check . --fix
```

### Single vs Double Quotes

Regarding single or double quotes: [PEP 8](https://peps.python.org/pep-0008/) doesn't give any recommendation, so we simply chose single quotes and sticked with it.
On qwerty keyboards it's a bit easier to type, is visually less cluttered, and it works well for strings containing double quotes from the English language.
Pre-commit makes sure, that you don't commit unnecessary double quotes.

### F-Strings

We use f-strings where ever possible because they are generally more readable - once you get used to them.
There are only a few places in the code base where performance really matters and f-strings might not be the best choice.
These places should be marked with a `# NOTE: ...` comment when diverging from f-string usage.

## Running tests

Our tests are built with pytest and pytest plugins that are installed with the poetry group `dev`.

Before submitting a pull request, please make sure that all tests are passing.
To run them all, use the following command in the root directory of RoSys:

```bash
pytest
```

## Documentation

### Formatting

Because it has [numerous benefits](https://nick.groenen.me/notes/one-sentence-per-line/) we write each sentence in a new line.

### Examples

Besides the written documentation we collect useful, compact stand-alone examples.
Each example should be about one concept.
Please try to make them as minimal as possible to show what is needed to get some kind of functionality.
We are happy to merge pull requests with new examples which show new concepts, ideas or interesting use cases.
To list your addition on the website itself, you can use the `click-and-drive` [example](https://github.com/zauberzeug/rosys/blob/f33d2feceb2f83d85f4f596095c018586b50c9a6/docs/examples/click-and-drive.md) as a blueprint.

## Pull requests

To get started, fork the repository on GitHub, clone it somewhere on your filesystem, commit and push your changes,
and then open a pull request (PR) with a detailed description of the changes you've made
(the PR button is shown on the GitHub website of your forked repository).

When submitting a PR, please make sure that the code follows the existing coding style and that all tests are passing.
If you're adding a new feature, please include tests that cover the new functionality.

## Thank you!

Thank you for your interest in contributing to NiceGUI!
We're looking forward to working with you!
