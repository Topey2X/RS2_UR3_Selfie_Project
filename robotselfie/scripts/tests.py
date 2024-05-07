from lib_img_processing import *  # noqa: F403
# from OpenCVTest.lib_img_processing import *

import cv2
import numpy as np
import matplotlib.pyplot as plt

## Generative AI was used (under supervision, and corrected at times) for the creation of these tests.


def test_face_detection() -> bool:
    print("\nTEST 1: FACIAL RECOGNITION TEST")
    minimum_accuracy = 90.0
    # try:
    images = [
        (
            "OpenCVTest/tests/test1/img1_test.jpg",
            "OpenCVTest/tests/test1/img1_truth.png",
        ),
        (
            "OpenCVTest/tests/test1/img2_test.jpg",
            "OpenCVTest/tests/test1/img2_truth.png",
        ),
        (
            "OpenCVTest/tests/test1/img3_test.jpg",
            "OpenCVTest/tests/test1/img3_truth.png",
        ),
        (
            "OpenCVTest/tests/test1/img4_test.jpg",
            "OpenCVTest/tests/test1/img4_truth.png",
        ),
        (
            "OpenCVTest/tests/test1/img5_test.jpg",
            "OpenCVTest/tests/test1/img5_truth.png",
        ),
    ]
    accuracy = []
    images_output = []
    def inside(rect, x, y):
        return rect[0] <= x <= rect[0] + rect[2] and rect[1] <= y <= rect[1] + rect[3]
    for i, (test_img_name, truth_img_name) in enumerate(images):
        try:
            test_img = cv2.imread(test_img_name)
            rect = identify_faces(
                convert_img_to_grayscale(test_img)
            )  # Expected to return (x, y, w, h)

            truth_img = cv2.imread(truth_img_name)  # BGR

            (x, y, w, h) = rect
            cv2.rectangle(test_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            images_output.append(test_img)

            # Initialize counters
            TP, FP, TN, FN = 0, 0, 0, 0
            w = truth_img.shape[1]
            h = truth_img.shape[0]
            
            for x in range(w):  # Image width
                for y in range(h):  # Image height
                    color = truth_img[y, x]

                    if (
                        color[1] == 255 and color[2] == 0 and color[0] == 0
                    ):  # Green pixel
                        if inside(rect, x, y):
                            TP += 1
                        else:
                            FN += 1
                    elif (
                        color[2] == 255 and color[1] == 0 and color[0] == 0
                    ):  # Red pixel
                        if inside(rect, x, y):
                            FP += 1
                        else:
                            TN += 1

            a = ((TP + TN) / (TP + TN + FN + FP)) * 100
            accuracy.append(a)
            # Print results
            print(f"\tSUBTEST {i+1}: {a}% ({TP}-{FP}/{TN}-{FN})")
        except Exception as e:
            # print(f"\tSUBTEST FAILED: Error ({repr(e)})")
            accuracy.append(0.0)
            print(f"\tSUBTEST FAILED: Error")

    worst_accuracy = min(accuracy)
    result = worst_accuracy >= minimum_accuracy
    print(f"TEST {'PASSED' if result else 'FAILED'} (worst case {worst_accuracy}%)")

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Face Detection Test")
    fign = 1
    columns = 3
    rows = 2
    for i, img in enumerate(images_output):
        fig.add_subplot(rows, columns, fign)
        fign += 1
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    return result


def test_edge_detection() -> bool:
    print("\nTEST 2: EDGE DETECTION TEST")
    minimum_accuracy = 90.0
    images = [
        (
            "OpenCVTest/tests/test2/img1_test.png",
            "OpenCVTest/tests/test2/img1_truth.png",
        ),
        (
            "OpenCVTest/tests/test2/img2_test.png",
            "OpenCVTest/tests/test2/img2_truth.png",
        ),
        (
            "OpenCVTest/tests/test2/img3_test.png",
            "OpenCVTest/tests/test2/img3_truth.png",
        ),
    ]
    accuracy = []
    images_output = []

    for i, (test_img_name, truth_img_name) in enumerate(images):
        try:
            # Load images
            test_img = cv2.imread(test_img_name)
            truth_img = cv2.imread(truth_img_name)

            # Process the test image to detect edges
            binary_img = extract_edges_from_face(test_img, 80, 130)

            # Prepare the mask for green pixels in truth image
            green_mask = (
                (truth_img[:, :, 1] == 255)
                & (truth_img[:, :, 0] == 0)
                & (truth_img[:, :, 2] == 0)
            )

            # Calculate accuracy
            white_pixels = binary_img == 255
            correct_pixels = np.logical_and(white_pixels, green_mask)
            a = (np.sum(correct_pixels) / np.sum(white_pixels)) * 100
            accuracy.append(a)

            # Output the subtest result
            print(f"\tSUBTEST {i+1}: {a}%")
            images_output.append((test_img, binary_img, truth_img))
        except Exception:
            accuracy.append(0.0)
            print("\tSUBTEST FAILED: Error")

    # Determine the overall test result
    worst_accuracy = min(accuracy)
    result = worst_accuracy >= minimum_accuracy
    print(f"TEST {'PASSED' if result else 'FAILED'} (worst case {worst_accuracy}%)")

    # Display the images
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Edge Detection Test")
    fign = 1
    columns = 3
    rows = len(images)
    for test_img, binary_img, truth_img in images_output:
        fig.add_subplot(rows, columns, fign)
        plt.imshow(cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB))
        fign += 1
        fig.add_subplot(rows, columns, fign)
        plt.imshow(binary_img, cmap="gray")
        fign += 1
        fig.add_subplot(rows, columns, fign)
        plt.imshow(cv2.cvtColor(truth_img, cv2.COLOR_BGR2RGB))
        fign += 1

    return result

def test_contour_extraction() -> bool:
    print("\nTEST 3: CONTOUR EXTRACTION TEST")
    minimum_successful = 75.0
    tests = [
        (
            "OpenCVTest/tests/test3/img1_test.png",
            [
                "OpenCVTest/tests/test3/img1_poly1.png",
            ],
        ),(
            "OpenCVTest/tests/test3/img2_test.png",
            [
                "OpenCVTest/tests/test3/img2_poly1.png",
                "OpenCVTest/tests/test3/img2_poly2.png",
                "OpenCVTest/tests/test3/img2_poly3.png",
                "OpenCVTest/tests/test3/img2_poly4.png",
            ],
        ),(
            "OpenCVTest/tests/test3/img3_test.png",
            [
                "OpenCVTest/tests/test3/img3_poly1.png",
                "OpenCVTest/tests/test3/img3_poly2.png",
                "OpenCVTest/tests/test3/img3_poly3.png",
                "OpenCVTest/tests/test3/img3_poly4.png",
            ],
        ),(
            "OpenCVTest/tests/test3/img4_test.png",
            [
                "OpenCVTest/tests/test3/img4_poly1.png",
                "OpenCVTest/tests/test3/img4_poly2.png",
                "OpenCVTest/tests/test3/img4_poly3.png",
                "OpenCVTest/tests/test3/img4_poly4.png",
                "OpenCVTest/tests/test3/img4_poly5.png",
            ],
        )
    ]
    accuracies = []
    images_output = []

    for test_idx, (test_img_name, poly_img_names) in enumerate(tests):
        try:
            # Load the test image
            test_img = cv2.imread(test_img_name, 0)  # Load in grayscale

            # Extract contours from the test image
            contours, _ = extract_contours_from_edges(test_img)

            # Load polygon images and prepare masks
            poly_masks = [cv2.imread(name, -1)[:, :, 3] for name in poly_img_names]  # Load alpha channel as mask

            # Check each contour against the polygon masks
            correct_contours = []
            incorrect_contours = []
            total_contours = len(contours)

            for contour in contours:
                in_any_polygon = any(
                    np.all(poly_mask[contour[:, 0, 1], contour[:, 0, 0]] > 0)
                    for poly_mask in poly_masks
                )
                if in_any_polygon:
                    correct_contours.append(contour)
                else:
                    incorrect_contours.append(contour)

            # Calculate and log accuracy for this test image
            accuracy = (
                (len(correct_contours) / total_contours) * 100
                if total_contours > 0
                else 100.0
            )
            accuracies.append(accuracy)
            print(f"\tSUBTEST {test_idx+1}: {accuracy}%")
            images_output.append((test_img, correct_contours, incorrect_contours))

        except Exception as e:
            accuracies.append(0.0)
            print(f"\tSUBTEST {test_idx+1} FAILED: Error {str(e)}")

    # Determine the overall test result
    # worst_accuracy = min(accuracies)
    successful = len([result for result in accuracies if result])/len(accuracies) * 100
    result = successful >= minimum_successful
    print(f"TEST {'PASSED' if result else 'FAILED'} ({successful}% Successful)")

    # Plot the results
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Contour Extraction Test Results")
    num_tests = len(images_output)
    
    # Define colormaps for correct and incorrect contours
    correct_cmap = plt.colormaps['Greens']
    incorrect_cmap = plt.colormaps['Reds']

    for idx, (test_img, correct_contours, incorrect_contours) in enumerate(images_output):
        # Plot original test image
        ax1 = fig.add_subplot(2, num_tests, idx + 1)
        plt.imshow(test_img, cmap='gray')
        ax1.title.set_text(f"Test {idx + 1}")
        plt.axis('off')

        # Plot contours on a black background
        ax2 = fig.add_subplot(2, num_tests, idx + 1 + num_tests)
        plt.imshow(np.zeros_like(test_img), cmap='gray')  # Black background
        total_correct = len(correct_contours)
        total_incorrect = len(incorrect_contours)
        
        correct_norm = plt.Normalize(vmin=0, vmax=total_correct-1)
        incorrect_norm = plt.Normalize(vmin=0, vmax=total_incorrect-1)
        
        # Plot each correct contour with a different green shade
        for i, contour in enumerate(correct_contours):
            color = correct_cmap(correct_norm(i)/2 + 0.5)
            print(color)
            plt.plot(contour[:, 0, 0], contour[:, 0, 1], color=color, linewidth=2)
            
        # Plot each incorrect contour with a different red shade
        for i, contour in enumerate(incorrect_contours):
            color = incorrect_cmap(incorrect_norm(i)/2 + 0.5)
            plt.plot(contour[:, 0, 0], contour[:, 0, 1], color=color, linewidth=2)
            
        ax2.title.set_text(f"Contours: {total_correct + total_incorrect}/{len(tests[idx][1])}")

    return result

def test_optimisation() -> bool:
    print("\nTEST 4: INITIAL OPTIMISATION TEST")
    threshold = 7
    minimum_accuracy = 100.0
    dataset = [
        [
            [[77, 44], [88, 228], [346, 475], [91, 362]],
            [[420, 427], [214, 352], [86, 398]],
            [[159, 107], [393, 435], [480, 39], [382, 144]],
            [[286, 326], [392, 404], [183, 340]],
            [[148, 196], [51, 385], [431, 168], [153, 350]],
            [[463, 229], [347, 286], [189, 16], [436, 73]],
            [[476, 384], [58, 130], [432, 400], [6, 240]],
            [[25, 366], [113, 431], [41, 253]],
            [[30, 117], [339, 160], [180, 269], [128, 457]],
            [[54, 375], [163, 149], [368, 258]],
            [
                [24, 408],
                [0, 77],
                [69, 445],
                [137, 138],
                [307, 184],
                [466, 257],
                [153, 24],
                [369, 330],
                [165, 446],
                [470, 69],
                [268, 324],
                [113, 171],
                [1, 298],
                [349, 347],
                [16, 105],
                [141, 206],
                [38, 428],
                [451, 156],
                [366, 349],
                [106, 180],
                [460, 319],
                [11, 199],
                [376, 118],
                [330, 244],
                [88, 140],
                [197, 429],
                [71, 179],
                [42, 153],
            ],
            [
                [297, 238],
                [302, 148],
                [67, 233],
                [185, 212],
                [82, 379],
                [107, 161],
                [462, 82],
                [223, 46],
                [308, 455],
                [347, 300],
                [193, 193],
                [16, 161],
                [266, 355],
                [130, 114],
                [37, 189],
                [261, 421],
                [320, 477],
                [29, 288],
                [335, 429],
                [139, 47],
                [46, 486],
                [102, 30],
                [14, 347],
                [110, 211],
            ],
        ],
        [
            [[184, 257], [437, 118], [427, 225], [351, 344]],
            [[127, 71], [141, 17], [126, 334]],
            [[133, 58], [402, 2], [54, 368]],
            [[476, 274], [213, 21], [102, 187]],
            [[488, 314], [445, 77], [439, 362], [379, 329]],
            [[23, 381], [367, 404], [496, 35]],
            [[213, 471], [481, 381], [277, 300], [457, 443]],
            [[350, 309], [461, 339], [226, 312]],
            [[244, 483], [24, 381], [338, 296], [320, 58]],
            [[207, 437], [126, 216], [187, 332], [19, 338]],
            [
                [341, 109],
                [200, 118],
                [247, 275],
                [261, 325],
                [307, 427],
                [44, 344],
                [34, 101],
                [347, 112],
                [347, 256],
                [471, 441],
                [161, 458],
                [143, 122],
                [8, 313],
                [198, 392],
                [143, 56],
                [153, 407],
                [481, 152],
                [159, 438],
                [434, 394],
                [6, 315],
                [253, 460],
                [26, 365],
                [242, 215],
            ],
            [
                [103, 340],
                [215, 134],
                [122, 400],
                [312, 222],
                [97, 103],
                [85, 264],
                [300, 264],
                [462, 485],
                [472, 425],
                [417, 16],
                [63, 140],
                [383, 469],
                [364, 83],
                [141, 357],
                [211, 459],
                [468, 464],
                [425, 298],
                [66, 278],
                [138, 341],
                [167, 52],
                [128, 460],
                [305, 379],
                [416, 41],
                [458, 392],
            ],
        ],
    ]
    accuracy = []
    returned_contours = []
    for i, contours in enumerate(dataset):
        retained_contours = eliminate_negligent_contours(contours, threshold)
        returned_contours.append(retained_contours)
        
        # Metrics calculation
        TP, FP, TN, FN = 0, 0, 0, 0
        for cnt in contours:
            if len(cnt) < threshold:
                if cnt in retained_contours:
                    FP += 1
                else:
                    TN += 1
            else:
                if cnt in retained_contours:
                    TP += 1
                else:
                    FN += 1
                    
        a = ((TP + TN) / (TP + TN + FN + FP)) * 100
        accuracy.append(a)
        # Print results
        print(f"\tSUBTEST {i+1}: {a}% ({TP}-{FP}/{TN}-{FN})")
        
    # Determine the overall test result
    worst_accuracy = min(accuracy)
    result = worst_accuracy >= minimum_accuracy
    print(f"TEST {'PASSED' if result else 'FAILED'} (worst case {worst_accuracy}%)")

    # Plotting results        
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Contour Culling Test Results")
    num_tests = len(dataset)
    for idx in range(num_tests):
        # Plot original test image
        contours = dataset[idx]
        ax1 = fig.add_subplot(2, num_tests, idx + 1)  # Unique index for each original contour plot
        plt.imshow(np.zeros((500, 500, 3)), cmap='gray')  # Black background
        ax1.title.set_text(f"Original Contours {idx + 1}")
        for contour in contours:
            np_contour = np.array(contour)
            color = 'g-' if len(contour) >= 7 else 'r-'
            plt.plot(np_contour[:, 0], np_contour[:, 1], color, linewidth=2)

        # Plot culled contours on a black background
        contours = returned_contours[idx]
        ax2 = fig.add_subplot(2, num_tests, num_tests + idx + 1)  # Unique index for each culled contour plot
        plt.imshow(np.zeros((500, 500, 3)), cmap='gray')  # Black background
        ax2.title.set_text(f"Culled Contours {idx + 1}")
        for contour in contours:
            np_contour = np.array(contour)
            color = 'g-' if len(contour) >= 7 else 'r-'
            plt.plot(np_contour[:, 0], np_contour[:, 1], color, linewidth=2)
    
    return result


def test_background_removal() -> bool:
    print("\nTEST 5: BACKGROUND REMOVAL TEST")
    minimum_accuracy = 80.0
    threshold = 0xC8
    images = [
        (
            "OpenCVTest/tests/test5/img1_test.jpg",
            "OpenCVTest/tests/test5/img1_truth.png",
        ),
        (
            "OpenCVTest/tests/test5/img2_test.jpg",
            "OpenCVTest/tests/test5/img2_truth.png",
        ),
        (
            "OpenCVTest/tests/test5/img3_test.jpg",
            "OpenCVTest/tests/test5/img3_truth.png",
        ),
        (
            "OpenCVTest/tests/test5/img4_test.jpg",
            "OpenCVTest/tests/test5/img4_truth.png",
        ),
    ]
    accuracy = []
    images_output = []

    for i, (test_img_name, truth_img_name) in enumerate(images):
        try:
            test_img = cv2.imread(test_img_name, cv2.IMREAD_UNCHANGED)
            result_img = remove_background(test_img)

            truth_img = cv2.imread(
                truth_img_name, cv2.IMREAD_UNCHANGED
            )  # BGR and alpha

            images_output.append((test_img, result_img))

            TP, FP, TN, FN = 0, 0, 0, 0
            w = truth_img.shape[1]
            h = truth_img.shape[0]
            for x in range(w):
                for y in range(h):
                    alpha = result_img[y, x, 3]  # Alpha channel of the output
                    color = truth_img[y, x]

                    if (
                        color[1] == 255 and color[2] == 0 and color[0] == 0
                    ):  # Green pixel
                        if alpha >= threshold:
                            TP += 1
                        else:
                            FN += 1
                    elif (
                        color[2] == 255 and color[1] == 0 and color[0] == 0
                    ):  # Red pixel
                        if alpha < threshold:
                            TN += 1
                        else:
                            FP += 1

            a = ((TP + TN) / (TP + TN + FN + FP)) * 100
            accuracy.append(a)
            print(f"\tSUBTEST {i+1}: {a}% ({TP}-{FP}/{TN}-{FN})")
        except Exception as e:
            accuracy.append(0.0)
            print(f"\tSUBTEST FAILED: Error")

    worst_accuracy = min(accuracy)
    result = worst_accuracy >= minimum_accuracy
    print(f"TEST {'PASSED' if result else 'FAILED'} (worst case {worst_accuracy}%)")

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Background Removal Test")
    columns = 4
    rows = 2
    fign = 1
    for i in range(rows):
        for img in images_output:
            ax = fig.add_subplot(rows, columns, fign)
            ax.imshow(
                cv2.cvtColor(img[i], cv2.COLOR_BGRA2RGBA)
            )  # Correcting for display
            fign += 1

    return result


def tests():
    tests = [
        # test_face_detection,  # Test 1: Face Detect
        # test_edge_detection,  # Test 2: Edge Detect
        test_contour_extraction,  # Test 3: Contour Detect
        # test_optimisation,  # Test 4: Optimisation
        # test_background_removal,  # Test 5: Background Removal
    ]
    results = [test() for test in tests]

    count = len([result for result in results if result])

    print(f"\n\nRESULTS:\n{count} tests passed, {len(tests)-count} tests failed")
    print(f'OVERALL RESULT: {"Pass" if count == len(tests) else "Fail"}')

    plt.show()


if __name__ == "__main__":
    tests()
