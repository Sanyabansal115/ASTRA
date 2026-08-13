import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'astra_llm_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config/openai.env is git-ignored and only exists locally; it is
        # installed when present so the lab-style key file keeps working.
        # config/*.env.local and other secrets are never installed.
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.env') + glob('config/*.env.example')),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        # Installed with pip in the workspace virtualenv, pinned pre-1.0 so
        # that AgentExecutor exists (same pins as the Week 12-13 labs):
        #   pip install "langchain<1.0.0" "langchain-mistralai<1.0.0"
    ],
    zip_safe=True,
    maintainer='Aboud Elzubair - LLM integration',
    maintainer_email='aabdal35@my.centennialcollege.ca',
    description='LLM (Mistral + LangChain) natural-language navigation agent for Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_nav_agent = astra_llm_agent.llm_nav_agent:main',
            'prompt_cli = astra_llm_agent.prompt_cli:main',
        ],
    },
)
