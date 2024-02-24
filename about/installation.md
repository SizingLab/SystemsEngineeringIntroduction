![](../images/header.jpg)

# Installation

The course is available on this repository:
https://github.com/SizingLab/SystemsEngineeringIntroduction

### Colab
Open [Google Colab](https://colab.research.google.com), open a new notebook and clone the repository by running in a cell:

`https://github.com/SizingLab/SystemsEngineeringIntroduction.git`


Then install the required dependencies by running in a cell:

`!pip install -r SystemsEngineeringIntroduction/requirements.txt`

### Local
Install [Anaconda](https://www.anaconda.com/download) environment.  

Download the zip file from [Github](https://github.com/SizingLab/SystemsEngineeringIntroduction) and unzip it.

Open a Anaconda Prompt terminal and `cd` to the recently unziped folder.

You can then create a new conda environment by running:

`conda create -n IntroIS python=3.9`

`conda activate IntroIS` or `source activate SystemsEngineeringIntroduction`

Then install required dependencies:
`pip install -r requirements.txt`

Run jupyter lab:
`jupyter lab`