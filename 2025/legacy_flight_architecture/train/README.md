Things to note before traning
-------------------------

### Folder meaning

There should be 3 folders, **train, val, test**, under the root directory of which the training script runs. <br/>

```
Train: for actually finetuning Yolo model.
Val:   for evaluating the model's performance after tuning
Test:  for testing the model's function. (Optional)
```

There should also be 2 folders, **images, labels**, in each of the 3 folders **train, val, test**. <br/>

```
Images: to hold the actual training images.
Labels: to hold the annotation txt files.
```

For every image, there should be a annotation .txt file with the same name. <br/>
E.g. If there are 1000 .png files in **/image**, there should also be 1000 .txt files in **/labels**. <br/>

### Folder structure

If any changes to path are made, update the corresponding part in **data.yaml**.

```
                |  train  |  images
                |         |
                |         |  labels
                |         
training root   |  val    |  images
                |         |
                |         |  labels         
                |
                |  test   |  images
                          |
                          |  labels
```
