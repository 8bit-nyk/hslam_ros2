/*
 * Copyright (C) 2023, Inria
 * GRAPHDECO research group, https://team.inria.fr/graphdeco
 * All rights reserved.
 *
 * This software is free for non-commercial, research and evaluation use 
 * under the terms of the LICENSE.md file.
 *
 * For inquiries contact  george.drettakis@inria.fr
 * 
 * This file is Derivative Works of Gaussian Splatting,
 * created by Yan Song Hu in 2024,
 * as part of GSO.
 */



#include "3DGS/include/gaussian_model.h"




void GaussianModel::loadPly(std::filesystem::path ply_path)
{
    std::ifstream instream_binary(ply_path, std::ios::binary);
    if (!instream_binary.is_open() || instream_binary.fail())
        throw std::runtime_error("Fail to open ply file at " + ply_path.string());
    instream_binary.seekg(0, std::ios::beg);

    tinyply::PlyFile ply_file;
    ply_file.parse_header(instream_binary);

    std::cout << "\t[ply_header] Type: " << (ply_file.is_binary_file() ? "binary" : "ascii") << std::endl;
    for (const auto & c : ply_file.get_comments())
        std::cout << "\t[ply_header] Comment: " << c << std::endl;
    for (const auto & c : ply_file.get_info())
        std::cout << "\t[ply_header] Info: " << c << std::endl;

    for (const auto &e : ply_file.get_elements()) {
        std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
        for (const auto &p : e.properties) {
            std::cout << "\t[ply_header] \tproperty: " << p.name << " (type=" << tinyply::PropertyTable[p.propertyType].str << ")";
            if (p.isList)
                std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
            std::cout << std::endl;
        }
    }

    std::shared_ptr<tinyply::PlyData> xyz, f_dc, f_rest, opacity, scales, rot;

    try { xyz = ply_file.request_properties_from_element("vertex", { "x", "y", "z" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { f_dc = ply_file.request_properties_from_element("vertex", { "f_dc_0", "f_dc_1", "f_dc_2" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    int n_f_rest = ((max_sh_degree_ + 1) * (max_sh_degree_ + 1) - 1) * 3;
    if (n_f_rest >= 0) {
        std::vector<std::string> f_rest_element_names(n_f_rest);
        for (int i = 0; i < n_f_rest; ++i)
            f_rest_element_names[i] = "f_rest_" + std::to_string(i);
        try {f_rest = ply_file.request_properties_from_element("vertex", f_rest_element_names); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }
    }

    try { opacity = ply_file.request_properties_from_element("vertex", { "opacity" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { scales = ply_file.request_properties_from_element("vertex", { "scale_0", "scale_1", "scale_2" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { rot = ply_file.request_properties_from_element("vertex", { "rot_0", "rot_1", "rot_2", "rot_3" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    ply_file.read(instream_binary);

    if (xyz)     std::cout << "\tRead " << xyz->count     << " total xyz "     << std::endl;
    if (f_dc)    std::cout << "\tRead " << f_dc->count    << " total f_dc "    << std::endl;
    if (f_rest)  std::cout << "\tRead " << f_rest->count  << " total f_rest "  << std::endl;
    if (opacity) std::cout << "\tRead " << opacity->count << " total opacity " << std::endl;
    if (scales)  std::cout << "\tRead " << scales->count  << " total scales "  << std::endl;
    if (rot)     std::cout << "\tRead " << rot->count     << " total rot "     << std::endl;

    // Data to std::vector
    const int num_points = xyz->count;

    const std::size_t n_xyz_bytes = xyz->buffer.size_bytes();
    std::vector<float> xyz_vector(xyz->count * 3);
    std::memcpy(xyz_vector.data(), xyz->buffer.get(), n_xyz_bytes);

    const std::size_t n_f_dc_bytes = f_dc->buffer.size_bytes();
    std::vector<float> f_dc_vector(f_dc->count * 3);
    std::memcpy(f_dc_vector.data(), f_dc->buffer.get(), n_f_dc_bytes);

    const std::size_t n_f_rest_bytes = f_rest->buffer.size_bytes();
    std::vector<float> f_rest_vector(f_rest->count * n_f_rest);
    std::memcpy(f_rest_vector.data(), f_rest->buffer.get(), n_f_rest_bytes);

    const std::size_t n_opacity_bytes = opacity->buffer.size_bytes();
    std::vector<float> opacity_vector(opacity->count * 1);
    std::memcpy(opacity_vector.data(), opacity->buffer.get(), n_opacity_bytes);

    const std::size_t n_scales_bytes = scales->buffer.size_bytes();
    std::vector<float> scales_vector(scales->count * 3);
    std::memcpy(scales_vector.data(), scales->buffer.get(), n_scales_bytes);

    const std::size_t n_rot_bytes = rot->buffer.size_bytes();
    std::vector<float> rot_vector(rot->count * 4);
    std::memcpy(rot_vector.data(), rot->buffer.get(), n_rot_bytes);

    // std::vector to torch::Tensor
    this->xyz_ = torch::from_blob(
        xyz_vector.data(), {num_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    this->features_dc_ = torch::from_blob(
        f_dc_vector.data(), {num_points, 3, 1},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_).transpose(1, 2).contiguous();

    this->features_rest_ = torch::from_blob(
        f_rest_vector.data(), {num_points, 3, n_f_rest / 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_).transpose(1, 2).contiguous();

    this->opacity_ = torch::from_blob(
        opacity_vector.data(), {num_points, 1},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    this->scaling_ = torch::from_blob(
        scales_vector.data(), {num_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    this->rotation_ = torch::from_blob(
        rot_vector.data(), {num_points, 4},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    GAUSSIAN_MODEL_TENSORS_TO_VEC

    this->active_sh_degree_ = this->max_sh_degree_;
}


void GaussianModel::savePly(std::filesystem::path result_path)
{
    // Prepare data to write
    torch::Tensor xyz = this->xyz_.detach().cpu();
    torch::Tensor normals = torch::zeros_like(xyz);
    torch::Tensor f_dc = this->features_dc_.detach().transpose(1, 2).flatten(1).contiguous().cpu();
    torch::Tensor f_rest = this->features_rest_.detach().transpose(1, 2).flatten(1).contiguous().cpu();
    torch::Tensor opacities = this->opacity_.detach().cpu();
    torch::Tensor scale = this->scaling_.detach().cpu();
    torch::Tensor rotation = this->rotation_.detach().cpu();

    std::filebuf fb_binary;
    fb_binary.open(result_path, std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + result_path.string());

    tinyply::PlyFile result_file;

    // xyz
    result_file.add_properties_to_element(
        "vertex", {"x", "y", "z"},
        tinyply::Type::FLOAT32, xyz.size(0),
        reinterpret_cast<uint8_t*>(xyz.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // normals
    result_file.add_properties_to_element(
        "vertex", {"nx", "ny", "nz"},
        tinyply::Type::FLOAT32, normals.size(0),
        reinterpret_cast<uint8_t*>(normals.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // f_dc
    std::size_t n_f_dc = this->features_dc_.size(1) * this->features_dc_.size(2);
    std::vector<std::string> property_names_f_dc(n_f_dc);
    for (int i = 0; i < n_f_dc; ++i)
        property_names_f_dc[i] = "f_dc_" + std::to_string(i);

    result_file.add_properties_to_element(
        "vertex", property_names_f_dc,
        tinyply::Type::FLOAT32, this->features_dc_.size(0),
        reinterpret_cast<uint8_t*>(f_dc.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // f_rest
    std::size_t n_f_rest = this->features_rest_.size(1) * this->features_rest_.size(2);
    std::vector<std::string> property_names_f_rest(n_f_rest);
    for (int i = 0; i < n_f_rest; ++i)
        property_names_f_rest[i] = "f_rest_" + std::to_string(i);

    result_file.add_properties_to_element(
        "vertex", property_names_f_rest,
        tinyply::Type::FLOAT32, this->features_rest_.size(0),
        reinterpret_cast<uint8_t*>(f_rest.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // opacities
    result_file.add_properties_to_element(
        "vertex", {"opacity"},
        tinyply::Type::FLOAT32, opacities.size(0),
        reinterpret_cast<uint8_t*>(opacities.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // scale
    std::size_t n_scale = scale.size(1);
    std::vector<std::string> property_names_scale(n_scale);
    for (int i = 0; i < n_scale; ++i)
        property_names_scale[i] = "scale_" + std::to_string(i);

    result_file.add_properties_to_element(
        "vertex", property_names_scale,
        tinyply::Type::FLOAT32, scale.size(0),
        reinterpret_cast<uint8_t*>(scale.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // rotation
    std::size_t n_rotation = rotation.size(1);
    std::vector<std::string> property_names_rotation(n_rotation);
    for (int i = 0; i < n_rotation; ++i)
        property_names_rotation[i] = "rot_" + std::to_string(i);

    result_file.add_properties_to_element(
        "vertex", property_names_rotation,
        tinyply::Type::FLOAT32, rotation.size(0),
        reinterpret_cast<uint8_t*>(rotation.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // Write the file
    result_file.write(outstream_binary, true);

    fb_binary.close();
}

void GaussianModel::saveSparsePointsPly(std::filesystem::path result_path)
{
    // Prepare data to write
    torch::Tensor xyz = this->sparse_points_xyz_.detach().cpu();
    torch::Tensor normals = torch::zeros_like(xyz);
    torch::Tensor color = (this->sparse_points_color_ * 255.0f).toType(torch::kUInt8).detach().cpu();

    std::filebuf fb_binary;
    fb_binary.open(result_path, std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + result_path.string());

    tinyply::PlyFile result_file;

    // xyz
    result_file.add_properties_to_element(
        "vertex", {"x", "y", "z"},
        tinyply::Type::FLOAT32, xyz.size(0),
        reinterpret_cast<uint8_t*>(xyz.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // normals
    result_file.add_properties_to_element(
        "vertex", {"nx", "ny", "nz"},
        tinyply::Type::FLOAT32, normals.size(0),
        reinterpret_cast<uint8_t*>(normals.data_ptr<float>()),
        tinyply::Type::INVALID, 0);

    // color
    result_file.add_properties_to_element(
        "vertex", {"red", "green", "blue"},
        tinyply::Type::UINT8, color.size(0),
        reinterpret_cast<uint8_t*>(color.data_ptr<uint8_t>()),
        tinyply::Type::INVALID, 0);

    // Write the file
    result_file.write(outstream_binary, true);

    fb_binary.close();
}