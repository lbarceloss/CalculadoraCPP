#pragma once
#include <cmath>

class Vector3D {
public:
    double x, y, z;

    // Construtor padrão
    Vector3D(double x = 0.0, double y = 0.0, double z = 0.0)
        : x(x), y(y), z(z) {
    }

    // Construtor de cópia
    Vector3D(const Vector3D& v) : x(v.x), y(v.y), z(v.z) {}

    // Normaliza o vetor
    Vector3D Normalize() {
        return DivideScalar(Length());
    }

    // Multiplica o vetor por um escalar
    Vector3D& MultiplyScalar(double value) {
        x *= value;
        y *= value;
        z *= value;
        return *this;
    }

    // Soma dois vetores
    Vector3D& Add(const Vector3D& vector3d) {
        x += vector3d.x;
        y += vector3d.y;
        z += vector3d.z;
        return *this;
    }

    // Adiciona valores individuais aos componentes
    Vector3D& Add3D(double dx, double dy, double dz) {
        x += dx;
        y += dy;
        z += dz;
        return *this;
    }

    // Subtrai um vetor do vetor atual
    Vector3D& Sub(const Vector3D& vector3d) {
        x -= vector3d.x;
        y -= vector3d.y;
        z -= vector3d.z;
        return *this;
    }

    // Subtrai valores individuais dos componentes
    Vector3D& Sub3D(double dx, double dy, double dz) {
        x -= dx;
        y -= dy;
        z -= dz;
        return *this;
    }

    // Divide o vetor por um escalar
    Vector3D& DivideScalar(double value) {
        if (value != 0.0) {
            double scalar = 1.0 / value;
            x *= scalar;
            y *= scalar;
            z *= scalar;
        }
        else {
            x = y = z = 0.0;
        }
        return *this;
    }

    // Produto vetorial (cross product)
    Vector3D& Cross(const Vector3D& vector3d) {
        double tempX = x, tempY = y, tempZ = z;
        x = tempY * vector3d.z - tempZ * vector3d.y;
        y = tempZ * vector3d.x - tempX * vector3d.z;
        z = tempX * vector3d.y - tempY * vector3d.x;
        return *this;
    }

    // Retorna o comprimento (magnitude) do vetor
    double Length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Operadores para facilitar manipulação de vetores
    Vector3D operator+(const Vector3D& v) const {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }

    Vector3D operator-(const Vector3D& v) const {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }

    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    Vector3D operator/(double scalar) const {
        return (scalar != 0.0) ? Vector3D(x / scalar, y / scalar, z / scalar) : Vector3D(0.0, 0.0, 0.0);
    }

    // Operador de atribuição
    Vector3D& operator=(const Vector3D& v) {
        if (this != &v) {
            x = v.x;
            y = v.y;
            z = v.z;
        }
        return *this;
    }
};
