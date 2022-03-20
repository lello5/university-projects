import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import datetime
import cv2
import math
import glob
import os
import argparse
from tqdm import tqdm

class HParams():
    # benchmark_name: used to draw the obstacles
    benchmark_name = ''
    # whether to plot all configurations
    print_full_path = False
    # wheter to print the path
    print_path = True
    # borders of the environment
    x_lim = (-3, 3)
    y_lim = (-3, 3)
    # fraction to enlarge image frame
    frame_frac = 1.0
    # car parameters
    car_width = 0.125
    car_height = 0.25
    m1 = 0.07
    # trailer parameters
    l_trailer = 0.26
    trailer_width = 0.125
    trailer_height = 0.26
    # steering parameters
    arrow_scale = 0.15
    arrow_head = 0.04
    # video parameters
    fps = 12
    
hparams = HParams()

# dictionary of circular obstacles parameters
# 	key: name of the obstacle
# 	value: triple (x, y, r), where (x, y) - coordinates of a circle center, r - radius
circles = {'O1': (0, 0, 1),
           'O2': (-1.5, 2, 0.5),
           'O3': (1, -2, 0.25),
           'O4': (-2, -1, 0.75),
           'O5': (2.2, 1, 0.3),
           'O6': (-2.1, 0.6, 0.7)}

# simplification: rectangular obstacles modeled as a set of circles
parking_circles = {'O1': (0.21, 0.9, 0.21),
                   'O2': (0.63, 0.9, 0.21),
                   'O3': (1.05, 0.9, 0.21),

                   'O4': (1.95, 0.9, 0.21),
                   'O5': (2.37, 0.9, 0.21),
                   'O6': (2.79, 0.9, 0.21),

                   'O7': (0.75, 0.5, 0.5),
                   'O8': (2.25, 0.5, 0.5),
                   'O9': (1.1, 0.2, 0.21),
                   'O10': (1.9, 0.2, 0.21)}

# draw circular obstacles scenario (custom environment)
def draw_circles(circles):
    for (key, value) in circles.items():
        circle = Circle(value[:-1], value[-1], facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5)
        plt.gca().add_patch(circle)
        label = plt.annotate(key, xy=value[:-1], fontsize=7, verticalalignment='center', horizontalalignment='center')

# draw three point turn scenario (from the paper)
def draw_turn():
    hparams.x_lim = (0, 3)
    hparams.y_lim = (0, 1)
    plt.gca().add_patch(Rectangle((0, 0.5), angle=0, width=1.125, height=0.5, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Rectangle((1.875, 0.5), angle=0, width=1.125, height=0.5, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    
# draw driver test scenario (from the paper)
def draw_real_parking():
    hparams.x_lim = (0, 3)
    hparams.y_lim = (0, 3)
    plt.gca().add_patch(Rectangle((0, 0), angle=0, width=1.25, height=1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))
    plt.gca().add_patch(Rectangle((1.75, 0), angle=0, width=1.25, height=1, facecolor=(0.6, 0.6, 0.6), edgecolor=(0, 0.8, 0.8), linewidth=0.5, alpha=0.5))

def solution_to_video(input_path: str,
                      video_path: str,
                      tree_path: str,
                      exp: str):
    if tree_path != '':
    	tree = plt.imread(tree_path)
    	print("Tree image was received.")
    
    if exp != '':
        hparams.benchmark_name = exp
    with open(input_path,'r') as f:
        lines = f.readlines()
        
        # according to the saving modality, lines[-2] = "\n" and lines[-1] = desired goal
        # saving starting configuration and final goal to always visualize them in the plot
        # start configuration qs
        x0, y0, theta0, psi0, phi0 = list(map(float, lines[0].split()))
        # REMEMBER: we need to save both the angle in degrees and in radians
        radAngle0 = -math.pi/2+theta0
        gradAngle0 = radAngle0*180/math.pi

        trailerAngle0 = theta0+psi0
        trailerRadAngle0 = -math.pi/2+trailerAngle0
        trailerGradAngle0 = trailerRadAngle0*180/math.pi

        # goal configuration qg
        xf, yf, thetaf, psif, phif = list(map(float, lines[-1].split()))
        radAnglef = -math.pi/2+thetaf
        gradAnglef = radAnglef*180/math.pi

        trailerAnglef = thetaf+psif
        trailerRadAnglef = -math.pi/2+trailerAnglef
        trailerGradAnglef = trailerRadAnglef*180/math.pi

        # discarding "\n" and the already-stored goal
        lines = lines[:-2]
        positions = []
        for row in tqdm(lines):
            if not hparams.print_full_path:
                # clear plot
                plt.clf()

            # reading the current configuration q
            x, y, theta, psi, phi = list(map(float, row.split()))
            radAngle = -math.pi/2+theta
            gradAngle = radAngle*180/math.pi

            trailerAngle = theta+psi
            trailerRadAngle = -math.pi/2+trailerAngle
            trailerGradAngle = trailerRadAngle*180/math.pi
            positions.append([x, y])
            
            # draw scenario
            if hparams.benchmark_name == 'obstacles':
                draw_circles(circles)
            if hparams.benchmark_name == 'turn':
                draw_turn()
            if hparams.benchmark_name == 'real':
                draw_real_parking()
                #draw_circles(parking_circles)

            # adjust visible area
            plt.xlim([hparams.frame_frac*hparams.x_lim[0], hparams.frame_frac*hparams.x_lim[1]])
            plt.ylim([hparams.frame_frac*hparams.y_lim[0], hparams.frame_frac*hparams.y_lim[1]])
            plt.gca().set_aspect('equal', adjustable='box')

            # creating the path
            if hparams.print_path:
                for p in positions:
                    plt.scatter(p[0], p[1], color = 'black', s=0.5)

            # creating the current car ...
            car_width = hparams.car_width
            car_height = hparams.car_height
            car = Rectangle((x - car_width/2 * math.cos(radAngle),
                             y - car_width/2 * math.sin(radAngle)), angle=gradAngle, width=car_width, height=car_height, linewidth=0.5, color='r', fill=True)
            plt.gca().add_patch(car)
            # ... plotting also qs and qg cars
            plt.gca().add_patch(Rectangle((x0 - car_width/2 * math.cos(radAngle0),
                                           y0 - car_width/2 * math.sin(radAngle0)), angle=gradAngle0, width=car_width, height=car_height, linewidth=0.5, color='r', fill=False))
            plt.gca().add_patch(Rectangle((xf - car_width/2 * math.cos(radAnglef),
                                           yf - car_width/2 * math.sin(radAnglef)), angle=gradAnglef, width=car_width, height=car_height, linewidth=0.5, color='r', fill=False))

            # this point is the midpoint of the rear axle of the current car ...
            plt.scatter(x, y, color = 'black', s=1)
            # ... plotting also qs and qg point
            plt.scatter(x0, y0, color = 'black', s=1)
            plt.scatter(xf, yf, color = 'black', s=1)

            # link representing wheels angle
            # for the current configuration ...
            arrow_scale = hparams.arrow_scale
            arrow_head = hparams.arrow_head
            plt.arrow(x+car_height*math.cos(theta), y+car_height*math.sin(theta), 
                      arrow_scale*math.cos(theta+phi), arrow_scale*math.sin(theta+phi), 
                      color='green', linewidth=0.5, head_width=arrow_head, length_includes_head=True)  
            # ... and for qs and qg
            plt.arrow(x0+car_height*math.cos(theta0), y0+car_height*math.sin(theta0), 
                      arrow_scale*math.cos(theta0+phi0), arrow_scale*math.sin(theta0+phi0), 
                      color='green', linewidth=0.5, head_width=arrow_head, length_includes_head=True, fill=False)  
            plt.arrow(xf+car_height*math.cos(thetaf), yf+car_height*math.sin(thetaf), 
                      arrow_scale*math.cos(thetaf+phif), arrow_scale*math.sin(thetaf+phif), 
                      color='green', linewidth=0.5, head_width=arrow_head, length_includes_head=True, fill=False)  

            # plotting the fixed link between the car and the trailer as a 2D line
            # for the current configuration ...
            m1 = hparams.m1
            plt.plot([x, x - m1 * math.cos(theta)],
                     [y, y - m1 * math.sin(theta)], color = 'black', linewidth=0.5)                 
            plt.scatter(x - m1 * math.cos(theta), y - m1 * math.sin(theta), color = 'black', s=1)
            # ... and for qs and qg
            plt.plot([x0, x0 - m1 * math.cos(theta0)],
                     [y0, y0 - m1 * math.sin(theta0)], color = 'black', linewidth=0.5)
            plt.scatter(x0 - m1 * math.cos(theta0), y0 - m1 * math.sin(theta0), color = 'black', s=1)
            plt.plot([xf, xf - m1 * math.cos(thetaf)],
                     [yf, yf - m1 * math.sin(thetaf)], color = 'black', linewidth=0.5)
            plt.scatter(xf - m1 * math.cos(thetaf), yf - m1 * math.sin(thetaf), color = 'black', s=1)

            # finally plotting the trailer as well
            l_trailer = hparams.l_trailer
            trailer_width = hparams.trailer_width
            trailer_height = hparams.trailer_height
            # for the current configuration ...
            xt = x - m1 * math.cos(theta) - l_trailer * math.cos(trailerAngle)
            yt = y - m1 * math.sin(theta) - l_trailer * math.sin(trailerAngle)
            plt.scatter(xt, yt, color = 'black', s=1)
            trailer = Rectangle((xt - trailer_width/2 * math.cos(trailerRadAngle),
                                 yt - trailer_width/2 * math.sin(trailerRadAngle)), angle=trailerGradAngle, width=trailer_width, height=trailer_height, linewidth=0.5, color='b', fill=True)
            plt.gca().add_patch(trailer)
            # ... for qs ...
            xt0 = x0 - m1 * math.cos(theta0) - l_trailer * math.cos(trailerAngle0)
            yt0 = y0 - m1 * math.sin(theta0) - l_trailer * math.sin(trailerAngle0)

            plt.scatter(xt0, yt0, color = 'black', s=1)
            plt.gca().add_patch(Rectangle((xt0 - trailer_width/2 * math.cos(trailerRadAngle0),
                                           yt0 - trailer_width/2 * math.sin(trailerRadAngle0)), angle=trailerGradAngle0, width=trailer_width, height=trailer_height, linewidth=0.5, color='b', fill=False))
            # ... and for qg
            xtf = xf - m1 * math.cos(thetaf) - l_trailer * math.cos(trailerAnglef)
            ytf = yf - m1 * math.sin(thetaf) - l_trailer * math.sin(trailerAnglef)

            plt.scatter(xtf, ytf, color = 'black', s=1)
            plt.gca().add_patch(Rectangle((xtf - trailer_width/2 * math.cos(trailerRadAnglef),
                                           ytf - trailer_width/2 * math.sin(trailerRadAnglef)), angle=trailerGradAnglef, width=trailer_width, height=trailer_height, linewidth=0.5, color='b', fill=False))

            if tree_path != '':
                plt.imshow(tree, extent=[hparams.frame_frac*hparams.x_lim[0], hparams.frame_frac*hparams.x_lim[1], hparams.frame_frac*hparams.y_lim[0], hparams.frame_frac*hparams.y_lim[1]])
            
            # print of the hitch angle    
            plt.text(hparams.frame_frac*hparams.x_lim[0]+0.1, hparams.frame_frac*hparams.y_lim[1]-0.1, "psi = " + str(round((psi*180/math.pi), 2)), ha='left', va = 'top')
            # saving current frame to later create the video
            savepath = str(datetime.datetime.now()).strip() + '.png'
            plt.savefig(savepath, dpi=200)

    print('Conversion to video...')
    
    # collecting every frame in an array
    img_array = []
    for filename in sorted(glob.glob('*.png')):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)
        os.remove(filename)

    # re-creating frames as a video in '.mp4' format
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), hparams.fps, size)
    for image in img_array:
        out.write(image)
    out.release()
    
    print('Conversion completed.')


def main():
    
    parser = argparse.ArgumentParser()
    
    # required parameters
    parser.add_argument(
        "--input_path",
        nargs='?',
        default='solution.txt',
        type=str,
        help='Path to the input txt file.'
    )
    parser.add_argument(
        "--video_path",
        nargs='?',
        default='solution_to_video.mp4',
        type=str,
        help='Path to the output video file.'
    )
    parser.add_argument(
        "--tree_path",
        nargs='?',
        default='',
        type=str,
        help='Path to the tree image.'
    )
    parser.add_argument(
        "--exp",
        nargs='?',
        default='',
        type=str,
        help='type of experiment: {forward, backward, obstacles, turn, real}'
    )
    args = parser.parse_args()
    
    print('Creating %s video from %s to %s' % (args.exp, args.input_path, args.video_path))
    
    solution_to_video(args.input_path, args.video_path, args.tree_path, args.exp)

if __name__ == '__main__':
    main()
