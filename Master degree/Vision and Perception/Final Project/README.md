# Vision & Perception project

This is the repository for the project of the **Vision & Perception** course.

Our work is based on the paper [SinGAN: Learning a Generative Model from a Single Natural Image](https://arxiv.org/abs/1905.01164) and contains a personal extension too. In particular, this project is based on:
- a re-implementation of the aforementioned original paper in *TensorFlow 2*, where we have created a version of the random generation process of a simple image containing balloons.

![ballons](./data/balloons.png)

- an extension of this process that focuses on the extreme case of face generation; we have tested three faces at three different scales and tried to replicate them taking into account the high importance of their global structure.

<img src="./data/face1.jpg"  width="250" height="250"> <img src="./data/face2.jpg"  width="250" height="250"> <img src="./data/face3.jpg"  width="250" height="250">

- a slightly different version of [Face Inpainting](https://paperswithcode.com/task/facial-inpainting), where our already-trained SinGAN fills the masked area of an image and tries to re-create realistic faces starting from the three original ones below.

<img src="./results/face_inpainting/face1/face1_masked.jpg"  width="250" height="250"> <img src="./results/face_inpainting/face2/face2_masked.jpg"  width="250" height="250"> <img src="./results/face_inpainting/face3/face3_masked.jpg"  width="250" height="250">

If you open the [notebook](./project.ipynb) on **Google Colab**, you are free to interact with our GUIs to set your own experiment or replicate our results, that are collected in the 'models' and 'results' folders available in this repository.

## Run the project

The project is self-contained in one single file where the user is free to interact by setting the modalities of the experiment in the proper GUI that is only visible with Google Colab.

### Training

In case of training, the user just needs to switch the **mode** option to *"train"* or *"train+test"*. We provide an interface to *TensorBoard* too to visualize the losses involved in the SinGAN learning.

We strongly suggest to adopt the following values for your experiments:
| Image | max_size | scale_attr |
| ---      | ---      | ---      |
| balloons   |  250   | 0.75   |
| face1   |  250   | 0.6   |
|  face1  | 365  |  0.55   |
|  face1  | 480  |  0.5   |
| face2   |  250   | 0.6   |
|  face2  | 365  |  0.55   |
|  face2  | 480  |  0.5   |
| face3   |  250   | 0.6   |
|  face3  | 365  |  0.55   |
|  face3  | 480  |  0.5   |

The model weights will be automatically saved to the corresponding directory in the 'models' folder.

### Testing

In case of testing, the user just needs to switch the **mode** option to *"test"* or *"train+test"*. We underline the fact that SinGAN needs to be already trained on the specific image you want to test.

## Results

### Random Samples


| Original image | Best results (for faces, one per scale) |
| ---      | ---  |
|  <img src="./data/balloons.png" width="200" height="150"> | <img src="./results/random_samples/balloons/result1.jpg" width="200" height="150"> <img src="./results/random_samples/balloons/result5.jpg" width="200" height="150"> <img src="./results/random_samples/balloons/result2.jpg" width="200" height="150"> |
| <img src="./data/face1.jpg"  width="200" height="200"> | <img src="./results/random_samples/face1/250/result2.jpg" width="200" height="200"> <img src="./results/random_samples/face1/365/result3.jpg" width="200" height="200"> <img src="./results/random_samples/face1/480/result2.jpg" width="200" height="200"> |
| <img src="./data/face2.jpg"  width="200" height="200"> | <img src="./results/random_samples/face2/250/result4.jpg" width="200" height="200"> <img src="./results/random_samples/face2/365/result4.jpg" width="200" height="200"> <img src="./results/random_samples/face2/480/result4.jpg" width="200" height="200">|
| <img src="./data/face3.jpg"  width="200" height="200"> | <img src="./results/random_samples/face3/250/result5.jpg" width="200" height="200"> <img src="./results/random_samples/face3/365/result1.jpg" width="200" height="200"> <img src="./results/random_samples/face3/480/result5.jpg" width="200" height="200"> |

### Face Inpainting

As already mentioned, this is a variation of the traditional Face Inpainting task: our model is not trained on uncompleted (or masked) images but, at testing phase, it just tries to fill the masked area and realize coherent faces.

| Masked image | Best results (for faces, one per scale) |
| ---      | ---  |
| <img src="./results/face_inpainting/face1/face1_masked.jpg"  width="200" height="200"> | <img src="./results/face_inpainting/face1/result1.jpg" width="200" height="200"> <img src="./results/face_inpainting/face1/result2.jpg" width="200" height="200"> <img src="./results/face_inpainting/face1/result3.jpg" width="200" height="200"> |
| <img src="./results/face_inpainting/face2/face2_masked.jpg"  width="200" height="200"> | <img src="./results/face_inpainting/face2/result1.jpg" width="200" height="200"> <img src="./results/face_inpainting/face2/result2.jpg" width="200" height="200"> <img src="./results/face_inpainting/face2/result3.jpg" width="200" height="200"> |
| <img src="./results/face_inpainting/face3/face3_masked.jpg"  width="200" height="200"> | <img src="./results/face_inpainting/face3/result1.jpg" width="200" height="200"> <img src="./results/face_inpainting/face3/result2.jpg" width="200" height="200"> <img src="./results/face_inpainting/face3/result3.jpg" width="200" height="200"> |

## Authors

#### - Lorenzo Nicoletti - 1797464

#### - Leandro Maglianella - 1792507


