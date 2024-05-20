import csv
# Thông tin kích thước ảnh, giấy và đường dẫn file
image_width = 450
image_height = 400
paper_width_mm = 297
paper_height_mm = 210
input_file = 'Waypoints.csv'
output_file = 'Waypoints_mm.csv'


def pixel_to_mm(input_file, output_file, image_width, image_height, paper_width_mm, paper_height_mm):
  """
  Chuyển đổi tọa độ điểm từ pixel sang mm và lưu vào file mới.

  Args:
      input_file (str): Đường dẫn đến file CSV đầu vào (x_pixels, y_pixels, z).
      output_file (str): Đường dẫn đến file CSV đầu ra (x_mm, y_mm, z).
      image_width (int): Chiều rộng ảnh theo pixel.
      image_height (int): Chiều cao ảnh theo pixel.
      paper_width_mm (float): Chiều rộng giấy theo mm.
      paper_height_mm (float): Chiều cao giấy theo mm.
  """

  # 1. Tính tỉ lệ
  scale_x = paper_width_mm / image_width
  scale_y = paper_height_mm / image_height
  # scale = min(scale_x, scale_y)  # Chọn tỉ lệ nhỏ nhất để ảnh vừa với giấy

  # 2. & 3. Đọc file CSV, chuyển đổi điểm và ghi vào file mới
  with open(input_file, 'r') as file_in, open(output_file, 'w', newline='') as file_out:
    reader = csv.reader(file_in)
    writer = csv.writer(file_out)
    for row in reader:
      if not row:  # Bỏ qua hàng trống
        continue
      x_pixels, y_pixels, z = map(float, row)
      x_mm = x_pixels * scale_x 
      y_mm = y_pixels * scale_y
      # writer.writerow([x_mm, y_mm, z])
      # Ghi dữ liệu với 4 chữ số thập phân
      file_out.write(f"{x_mm:.4f},{y_mm:.4f},{z}\n")

  # 4. & 5. Đọc file mới và in giá trị
with open(output_file, 'r') as file_out:
    reader = csv.reader(file_out)
    next(reader)  # Bỏ qua header
    row_num = 2
    for row in reader:
        x_mm, y_mm, z = map(float, row)  # Đọc cả giá trị z
        print(f"Hàng số {row_num}: giá trị x_mm là {x_mm}, giá trị y_mm là {y_mm} và giá trị z là {z}")
        
        row_num += 1



# Thực hiện chuyển đổi và in kết quả
pixel_to_mm(input_file, output_file, image_width, image_height, paper_width_mm, paper_height_mm)
print("Hoàn thành!")