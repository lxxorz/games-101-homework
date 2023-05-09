## Usage

Example:

```sh
cd ./Homework1/Assignment1
cmake ./
make
./Rasterizer -r 30 result.png
```
Result:

![](assets/example.png)

## Eigen Introduce

1. Creating a Zero Matrix
```c++
Eigen::Matrix4f m = Eigen::Matrix4f::Zero();

```

2. Creating a Identity Matrix
```c++
Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
```

3. Modifying Matrix Cell by Specifying Row and Column
```c++
m(row, col) = 0;
```
