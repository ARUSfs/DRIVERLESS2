import os
import sys
import recommonmark
from recommonmark.transform import AutoStructify

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ARUS Driverless'
copyright = '2023, ARUS'
author = 'ARUS'
release = '0.3.0'

# -- Importations ------------------------------------------------------------

#sys.path.insert(0, os.path.abspath('.'))
#sys.path.insert(0, os.path.abspath('./ejemplo'))

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
	'myst_parser',
	'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    'sphinx.ext.autosummary',
    'numpydoc',
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
#html_static_path = ['_static']
html_title = 'ARUS DRIVERLESS'
html_logo = 'https://images.squarespace-cdn.com/content/v1/5e45d3a8e509f61738454469/81e0d83b-0fa6-48d0-801b-1f0c1515d786/logo+arus+cuadrado+web.png'
html_favicon = 'https://images.squarespace-cdn.com/content/v1/5e45d3a8e509f61738454469/4d7d9627-f1a6-4210-bc4d-6e90076418d1/favicon.ico?format=100w'
html_show_sphinx = False
#master_doc = 'content/index'
source_suffix = {'.rst': 'restructuredtext', '.md': 'markdown'}


html_theme_options = {
    "sidebar_hide_name": True,
    "top_of_page_button": "edit",
    "footer_icons": [
        {
            "name": "GitHub",
            "url": "https://github.com/arusfs/arussim/tree/main",
            "html": """
                <svg stroke="currentColor" fill="currentColor" stroke-width="0" viewBox="0 0 16 16">
                    <path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"></path>
                </svg>
            """,
        },
    ],
}


source_parsers = {
    '.md': 'recommonmark.parser.CommonMarkParser',
}

html_use_smartypants = False
html_show_sphinx = False
html_scaled_image_link = False

def setup(app):
    app.add_config_value('recommonmark_config', {
        'enable_auto_toc_tree': True,
        'auto_toc_tree_section': 'Contents',
        'enable_eval_rst': True,
        'enable_auto_doc_ref': True,
    }, True)
    app.add_transform(AutoStructify)
