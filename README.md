## Usage

Example:

Select the specified assignment
```sh
cd ./Homework1/Assignment1
```

Build:
```sh
cmake ./
make
```
Excute:
```sh
./Rasterizer -r 30 result.png
```
Result:

![](assets/example.png)

## Introduction to Eigen

1. Creating a Zero Matrix
```c++
Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
```

2. Creating an Identity Matrix
```c++
Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
```

3. Modifying Matrix Cell by Specifying Row and Column
```c++
m(row, col) = 0;
```

### Assignment 1

1. the zNear and zFar is negative

### Assignment 2

1. Handle vectors with care, ensuring that the order follows "end point minus start point"
2. Float-point equality operation is not accurate.
