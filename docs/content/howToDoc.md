# How to document

## Introduction
- This page is built using the Sphinx framework, which allows generating HTML documentation from files in Markdown format. For more information about Sphinx, you can consult the [official documentation](https://www.sphinx-doc.org/en/master/).
- In this project, we will use Myst-Parser as the markup language, which is a markup language that allows writing documents in Markdown format, but with the possibility of adding Python, RST, etc. code. For more information about Myst-Parser, you can consult the [official documentation](https://myst-parser.readthedocs.io/en/latest/).

## Dependencies
- The GitHub repository is set up to automatically install the necessary dependencies to generate the HTML files and then publish them on GitHub Pages. Therefore, it is not necessary to install anything to generate the documentation, simply modify the Markdown (.md) files, which are located in the /docs folder of the [doc](https://github.com/ARUSfs/ARUSSim/tree/doc) branch, and upload them to the repository.
- However, you may want to install these dependencies if you want to see the result of your changes locally before uploading them to the repository. To do this, follow these steps:
    1. Install Python 3.8 or higher.
    2. Install pip.
    3. Install the necessary dependencies to generate the documentation:
        - `pip install -r requirements.txt`
    4. Generate the HTML files:
        - `make html`
    5. Open the index.html file located in the /build/html folder.
    6. Alternatively to point 5, you can open a local web server to view the documentation in the web browser (https://0.0.0.0:8000). To do this, run the following command in the /build/html folder:
        - `python3 -m http.server 8000`
    7. Another alternative to point 5 is to use sphinx-autobuild, which allows you to automatically generate the HTML files when you modify the Markdown (.md) files. To do this, run the following command in the /docs folder:
        - `pip install sphinx-autobuild` (only the first time)
        - `sphinx-autobuild . _build/html`


## How to document
- To add a new page to the documentation, create a new Markdown (.md) file in the /docs folder. This file must necessarily contain a title (#), and can contain other Markdown elements, such as lists, tables, etc. In addition, it can contain Python, RST, etc. code. For more information on how to write code in Markdown, you can consult the [official documentation](https://myst-parser.readthedocs.io/en/latest/using/syntax.html#syntax). 
- To familiarize yourself with the Myst-Parser syntax, it is recommended to use its [Live Preview](https://myst-parser.readthedocs.io/en/latest/live-preview.html) tool, which allows you to see the result of writing in Markdown in real-time.
