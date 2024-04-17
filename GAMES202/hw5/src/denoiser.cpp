#include "denoiser.h"

Denoiser::Denoiser() : m_useTemportal(false) {}

void Denoiser::Reprojection(const FrameInfo &frameInfo) {
    int height = m_accColor.m_height;
    int width = m_accColor.m_width;
    Matrix4x4 preWorldToScreen =
        m_preFrameInfo.m_matrix[m_preFrameInfo.m_matrix.size() - 1];
    Matrix4x4 preWorldToCamera =
        m_preFrameInfo.m_matrix[m_preFrameInfo.m_matrix.size() - 2];
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // TODO: Reproject
            m_valid(x, y) = false;
            m_misc(x, y) = Float3(0.f);
            int id = frameInfo.m_id(x, y);
            if (id == -1) {
                continue;
            }
            Matrix4x4 worldToLocal = Inverse(frameInfo.m_matrix[id]);
            Matrix4x4 preLocalToWorld = m_preFrameInfo.m_matrix[id];
            Matrix4x4 preCameraToScreen = m_preFrameInfo.m_matrix[id + 1];
            auto preLocalPos = worldToLocal(frameInfo.m_position(x, y), Float3::EType::Point);
            auto preWorldPos = preLocalToWorld(preLocalPos, Float3::EType::Point);
            auto preScreenPos = preWorldToScreen(preWorldPos, Float3::EType::Point);
            if (preScreenPos.x < 0 || preScreenPos.x >= width || preScreenPos.y < 0 || preScreenPos.y >= height) {
                continue;
            }
            if (id == m_preFrameInfo.m_id(preScreenPos.x, preScreenPos.y)) {
                m_misc(x, y) = m_accColor(preScreenPos.x, preScreenPos.y);
                m_valid(x, y) = true;
            }
        }
    }
    std::swap(m_misc, m_accColor);
}

void Denoiser::TemporalAccumulation(const Buffer2D<Float3> &curFilteredColor) {
    int height = m_accColor.m_height;
    int width = m_accColor.m_width;
    int kernelRadius = 3;
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // TODO: Temporal clamp
            Float3 color = m_accColor(x, y);
            // TODO: Exponential moving average
            float alpha = 1.0f;
            if (m_valid(x, y)) {
                alpha = m_alpha;
                Float3 mu(0.0f);
                Float3 sigma(0.0f);
                for (int i = std::max(0, x - kernelRadius); i <= std::min(width - 1, x + kernelRadius); i++) {
                    for (int j = std::max(0, y - kernelRadius); j <= std::min(height - 1, y + kernelRadius); j++) {
                        mu += curFilteredColor(i, j);
                        sigma += Sqr(curFilteredColor(x, y) - curFilteredColor(i, j));
                    }
                }
                mu /= (kernelRadius * 2 + 1) * (kernelRadius * 2 + 1);
                sigma = SafeSqrt(sigma) / (kernelRadius * 2 + 1);
                color = Clamp(color, mu - sigma * m_colorBoxK, mu + sigma * m_colorBoxK);
            }
            m_misc(x, y) = Lerp(color, curFilteredColor(x, y), alpha);
        }
    }
    std::swap(m_misc, m_accColor);
}

Buffer2D<Float3> Denoiser::Filter(const FrameInfo &frameInfo) {
    int height = frameInfo.m_beauty.m_height;
    int width = frameInfo.m_beauty.m_width;
    Buffer2D<Float3> filteredImage = CreateBuffer2D<Float3>(width, height);
    int kernelRadius = 16;
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // TODO: Joint bilateral filter
            Float3 result;
            float weightSum = 0.0f;
            for (int i = std::max(0, x - kernelRadius); i <= std::min(width - 1, x + kernelRadius); i++) {
                for (int j = std::max(0, y - kernelRadius); j <= std::min(height - 1, y + kernelRadius); j++) {
                    auto exp1 = SqrDistance(frameInfo.m_position(x, y), frameInfo.m_position(i, j)) / (2.0f * m_sigmaCoord * m_sigmaCoord);
                    auto exp2 = SqrDistance(frameInfo.m_beauty(x, y), frameInfo.m_beauty(i, j)) / (2.0f * m_sigmaColor * m_sigmaColor);
                    auto exp3 = SafeAcos(Dot(frameInfo.m_normal(x, y), frameInfo.m_normal(i, j))) / m_sigmaNormal;
                    exp3 = exp3 * exp3 / 2.0f;
                    auto exp4 = exp1 > 0.0f ? Dot(frameInfo.m_normal(x, y), Normalize(frameInfo.m_position(i, j) - frameInfo.m_position(x, y))) / m_sigmaPlane : 0.0f;
                    exp4 = exp4 * exp4 / 2.0f;
                    float weight = std::exp(-exp1 - exp2 - exp3 - exp4);
                    result += frameInfo.m_beauty(i, j) * weight;
                    weightSum += weight;
                }
            }
            // std::cout << "weightSum: " << weightSum << std::endl;
            filteredImage(x, y) = result / std::max(weightSum, 0.0001f);
        }
    }
    return filteredImage;
}

void Denoiser::Init(const FrameInfo &frameInfo, const Buffer2D<Float3> &filteredColor) {
    m_accColor.Copy(filteredColor);
    int height = m_accColor.m_height;
    int width = m_accColor.m_width;
    m_misc = CreateBuffer2D<Float3>(width, height);
    m_valid = CreateBuffer2D<bool>(width, height);
}

void Denoiser::Maintain(const FrameInfo &frameInfo) { m_preFrameInfo = frameInfo; }

Buffer2D<Float3> Denoiser::ProcessFrame(const FrameInfo &frameInfo) {
    // Filter current frame
    Buffer2D<Float3> filteredColor;
    filteredColor = Filter(frameInfo);

    // Reproject previous frame color to current
    if (m_useTemportal) {
        Reprojection(frameInfo);
        TemporalAccumulation(filteredColor);
    } else {
        Init(frameInfo, filteredColor);
    }

    // Maintain
    Maintain(frameInfo);
    if (!m_useTemportal) {
        m_useTemportal = true;
    }
    return m_accColor;
}
