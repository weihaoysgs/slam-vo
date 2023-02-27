- 如何通过系统的 clang-format 对代码进行格式化
  - https://zhuanlan.zhihu.com/p/514541589
  
  - 这里我们首先生成 Google 风格的 .clang-format 模板文件：
    `clang-format -style=google -dump-config > .clang-format`
    
  - 每个选项的具体含义可以在官方文档中找到。https://clang.llvm.org/docs/ClangFormatStyleOptions.html
  
  - 如果希望某个代码段不要参与格式化，可使用 // clang-format off 和 // clang-format on 注释对该代码段进行限定，像下面这样：
  
    ```c++
    void test()
    {
        // clang-format off
        Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 6);
        ref_mat <<
             1,        0,        0,        0,        0,        0,
             1,      0.5,     0.25,    0.125,   0.0625,  0.03125,
             1,        1,        1,        1,        1,        1,
            -1,       -0,       -0,       -0,       -0,       -0,
            -1,     -0.5,    -0.25,   -0.125,  -0.0625, -0.03125,
            -1,       -1,       -1,       -1,       -1,       -1;
        // clang-format on
    }
    ```
  
    