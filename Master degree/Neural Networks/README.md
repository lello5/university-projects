# COVID-ResNet-for-Neural-Networks-project
The goal of this work is to build open source and open access datasets of chest Xrays images and present an accurate Convolutional Neural Network framework for differentiating COVID19 cases from other pneumonia cases.


## Steps to generate the dataset

1. Download the datasets listed above
 * `git clone https://github.com/ieee8023/covid-chestxray-dataset.git`
 * `git clone https://github.com/agchung/Figure1-COVID-chestxray-dataset.git`
 * `git clone https://github.com/agchung/Actualmed-COVID-chestxray-dataset.git`
 * go to this [link](https://www.kaggle.com/tawsifurrahman/covid19-radiography-database) to download the COVID-19 Radiography database. Only the COVID-19 image folder and metadata file is required. The overlaps between covid-chestxray-dataset are handled
 * go to this [link](https://www.kaggle.com/c/rsna-pneumonia-detection-challenge/data) to download the RSNA pneumonia dataset
2. Create a `data` directory and within the data directory, create a `train` and `test` directory
3. Use [generate\_dataset.ipynb](/generate_dataset.ipynb) to combine the datasets to create a dataser for classification. Make sure to remember to change the file paths.
4. I provide the [train](/train_split.txt) and [test](/test_split.txt) txt files with patientId, image path and label. The labels are 'normal', 'pneumonia', and 'COVID-19' for the dataset. 

## Dataset data distribution

Chest radiography images distribution
|  Type | Normal | Pneumonia | COVID-19 | Total |
|:-----:|:------:|:---------:|:--------:|:-----:|
| train |  7966  |    5475   |   1645   | 15086 |
|  test |   885  |     594   |     67   |  1546 |

Patients distribution
|  Type | Normal | Pneumonia | COVID-19 |  Total |
|:-----:|:------:|:---------:|:--------:|:------:|
| train | 7966   |   5451    |   1483   |  14900 |
|  test |  885   |    591    |     46   |   1522 |

## Pre-processing
Use [preprocessing.ipynb](/preprocessing.ipynb) to generate the .npy files.

## Training and Evaluation
Use [project.ipynb](/project.ipynb) to train and evaluate the models.


## Report
Check the [report](/report.pdf) for more info.
