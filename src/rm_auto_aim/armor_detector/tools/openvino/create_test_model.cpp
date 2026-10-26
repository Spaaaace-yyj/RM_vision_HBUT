// Generate a tiny input-dependent fixture, not an accuracy/performance model.
#include <iostream>
#include <openvino/openvino.hpp>
#include <openvino/opsets/opset13.hpp>
#include <openvino/pass/serialize.hpp>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 1;
  try {
    namespace op = ov::opset13;
    const std::string prefix = argv[1];
    auto x = std::make_shared<op::Parameter>(ov::element::f32,
                                             ov::Shape{1, 3, 8, 8});
    auto axes =
        op::Constant::create(ov::element::i64, ov::Shape{4}, {0, 1, 2, 3});
    auto mean = std::make_shared<op::ReduceMean>(x, axes, false);
    auto shift = std::make_shared<op::Multiply>(
        mean, op::Constant::create(ov::element::f32, ov::Shape{}, {4.0F}));
    const std::vector<float> base{0, 1, 0, 6, 2, 6, 2, 1, 10, 5, 0,
                                  0, 0, 0, 0, 0, 5, 0, 0, 0,  0, 0};
    const std::vector<float> mask{1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    auto moved = std::make_shared<op::Multiply>(
        shift, op::Constant::create(ov::element::f32, ov::Shape{22}, mask));
    auto y = std::make_shared<op::Add>(
        op::Constant::create(ov::element::f32, ov::Shape{22}, base), moved);
    auto output = std::make_shared<op::Reshape>(
        y, op::Constant::create(ov::element::i64, ov::Shape{3}, {1, 1, 22}),
        false);
    auto model = std::make_shared<ov::Model>(ov::OutputVector{output},
                                             ov::ParameterVector{x});
    ov::serialize(model, prefix + ".xml", prefix + ".bin");
    auto bad_output = op::Constant::create(
        ov::element::f32, ov::Shape{1, 1, 21}, std::vector<float>(21, 0));
    auto bad_model = std::make_shared<ov::Model>(ov::OutputVector{bad_output},
                                                 ov::ParameterVector{x});
    ov::serialize(bad_model, prefix + "_bad.xml", prefix + "_bad.bin");
    std::cout << "Created input-dependent test fixtures\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
