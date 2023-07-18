import torch
import os
from network import modeling
from datasets import VOCSegmentation, Cityscapes, cityscapes
import cv2
import matplotlib.pyplot as plt

# model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet101', pretrained=True)
# state_dict=torch.load('best_deeplabv3_resnet101_voc_os16.pth')['model_state']

model = modeling.__dict__['deeplabv3_resnet101'](num_classes=21, output_stride=8)
model.load_state_dict( torch.load('../best_deeplabv3_resnet101_voc_os16.pth')['model_state'])

model.eval()

decode_fn = Cityscapes.decode_target

path='/home/dell/CV_WPI/venk_hw2/Code/pointcloud/images/'
# filename='../0000000224.png'

image_data = os.listdir(path)
image_data.sort()



from PIL import Image
from torchvision import transforms

for filename in image_data:

    print(filename)

    input_image = Image.open(path+filename)
    input_image = input_image.convert("RGB")

    # print(input_image.size)
    preprocess = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    input_tensor = preprocess(input_image)
    input_batch = input_tensor.unsqueeze(0) # create a mini-batch as expected by the model

    # print(input_batch.size())

    # move the input and model to GPU for speed if available
    if torch.cuda.is_available():
        input_batch = input_batch.to('cuda')
        model.to('cuda')

    with torch.no_grad():
        output = model(input_batch)
    output_predictions = output.max(1)[1].cpu().numpy()[0]

    colorized_preds = decode_fn(output_predictions).astype('uint8')
    # print(type(colorized_preds))
    # print(colorized_preds.shape)
    # cv2.imshow('file',colorized_preds)
    # cv2.waitKey(0)
    filenm=filename.split('.')[0]
    cv2.imwrite(f'../pointcloud/images_seg/{filenm}_seg.png',colorized_preds[:,:,::-1]) ### RGB to BGR
    
