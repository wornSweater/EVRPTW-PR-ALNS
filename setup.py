from setuptools import setup, find_packages

setup(
    name='EVRPTW_PR_ALNS',
    version='0.0.2',
    packages=find_packages(),
    install_requires=[
        'numpy==2.0.1',
        'gurobipy==11.0.3',
        'pandas==2.2.2',
    ],
    author='Xiaojing Lu',
    author_email='luxiaojing1204@outlook.com',
    description='EVRPTW_PR ALNS algorithm sovler and heuristic solution initializer with MILP feasibility checker',
    # long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/wornSweater/EVRPTW-PR-ALNS',
    classifiers=[
        'Programming Language :: Python :: 3',
    ],
    python_requires='>=3.8',
)