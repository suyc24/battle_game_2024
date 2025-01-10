#include "suyc.h"

#include "battle_game/core/bullets/bullets.h"
#include "battle_game/core/game_core.h"
#include "battle_game/graphics/graphics.h"

namespace battle_game::unit {

namespace {
uint32_t Edgar_body_model_index = 0xffffffffu;
uint32_t Edgar_turret_model_index = 0xffffffffu;
uint32_t Edgar_marker_model_index = 0xffffffffu;
}  // namespace

Edgar::Edgar(GameCore *game_core, uint32_t id, uint32_t player_id)
    : Unit(game_core, id, player_id) {
  if (!~Edgar_body_model_index) {
    auto mgr = AssetsManager::GetInstance();
    {
      /* Tank Body (Hexagon) */
      std::vector<ObjectVertex> body_vertices;
      std::vector<uint32_t> body_indices;
      const int sides = 6;
      const float radius = 1.2f;
      const float angle_increment = glm::two_pi<float>() / sides;
      for (int i = 0; i < sides; ++i) {
        float angle = angle_increment * i;
        body_vertices.push_back({{radius * std::cos(angle), radius * std::sin(angle)},
                                 {0.0f, 0.0f},
                                 {1.0f, 1.0f, 1.0f, 1.0f}});
      }
      // Add center vertex for fan rendering
      body_vertices.push_back({{0.0f, 0.0f},
                               {0.0f, 0.0f},
                               {1.0f, 1.0f, 1.0f, 1.0f}});
      for (int i = 0; i < sides; ++i) {
        body_indices.push_back(i);
        body_indices.push_back((i + 1) % sides);
        body_indices.push_back(sides); // Center vertex
      }
      Edgar_body_model_index = mgr->RegisterModel(body_vertices, body_indices);
    }

    {
      /* Tank Turret */
      std::vector<ObjectVertex> turret_vertices;
      std::vector<uint32_t> turret_indices;
      const int precision = 60;
      const float inv_precision = 1.0f / float(precision);
      for (int i = 0; i < precision; i++) {
        auto theta = (float(i) + 0.5f) * inv_precision;
        theta *= glm::pi<float>() * 2.0f;
        auto sin_theta = std::sin(theta);
        auto cos_theta = std::cos(theta);
        turret_vertices.push_back({{sin_theta * 0.5f, cos_theta * 0.5f},
                                   {0.0f, 0.0f},
                                   {0.7f, 0.7f, 0.7f, 1.0f}});
        turret_indices.push_back(i);
        turret_indices.push_back((i + 1) % precision);
        turret_indices.push_back(precision);
      }
      turret_vertices.push_back(
          {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.7f, 0.7f, 0.7f, 1.0f}});
      turret_vertices.push_back(
          {{-0.1f, 0.0f}, {0.0f, 0.0f}, {0.7f, 0.7f, 0.7f, 1.0f}});
      turret_vertices.push_back(
          {{0.1f, 0.0f}, {0.0f, 0.0f}, {0.7f, 0.7f, 0.7f, 1.0f}});
      turret_vertices.push_back(
          {{-0.1f, 1.2f}, {0.0f, 0.0f}, {0.7f, 0.7f, 0.7f, 1.0f}});
      turret_vertices.push_back(
          {{0.1f, 1.2f}, {0.0f, 0.0f}, {0.7f, 0.7f, 0.7f, 1.0f}});
      turret_indices.push_back(precision + 1 + 0);
      turret_indices.push_back(precision + 1 + 1);
      turret_indices.push_back(precision + 1 + 2);
      turret_indices.push_back(precision + 1 + 1);
      turret_indices.push_back(precision + 1 + 2);
      turret_indices.push_back(precision + 1 + 3);
      Edgar_turret_model_index =
          mgr->RegisterModel(turret_vertices, turret_indices);
    }

    /* Tank Marker */
    {
    std::vector<ObjectVertex> marker_vertices = {
          {{0.2f, 0.2f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f}},
          {{0.2f, -0.2f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f}},
          {{0.4f, 0.0f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f}}
      };
      std::vector<uint32_t> marker_indices = {0, 1, 2};
      Edgar_marker_model_index = mgr->RegisterModel(marker_vertices, marker_indices);
    }
  }
}

void Edgar::Render() {
  battle_game::SetTransformation(position_, rotation_);
  battle_game::SetTexture(0);
  battle_game::SetColor(game_core_->GetPlayerColor(player_id_));
  battle_game::DrawModel(Edgar_body_model_index);

  battle_game::SetTransformation(position_, turret_rotation_);
  battle_game::DrawModel(Edgar_turret_model_index);

  if (has_marker_) {
    battle_game::SetTransformation(position_, rotation_);
    battle_game::SetColor({1.0f, 0.0f, 0.0f, 1.0f});
    battle_game::DrawModel(Edgar_marker_model_index);
  }
}

void Edgar::Update() {
  TankMove(3.0f, glm::radians(180.0f));
  TurretRotate();
  Fire();
  HandleMarker();
  HandleBoost();
}

void Edgar::TankMove(float move_speed, float rotate_angular_speed) {
  auto player = game_core_->GetPlayer(player_id_);
  if (player) {
    auto &input_data = player->GetInputData();
    glm::vec2 offset{0.0f};
    if (input_data.key_down[GLFW_KEY_W]) {
      offset.y += 1.0f;
    }
    if (input_data.key_down[GLFW_KEY_S]) {
      offset.y -= 1.0f;
    }
    float speed = move_speed * GetSpeedScale();
    offset *= kSecondPerTick * speed;
    auto new_position =
        position_ + glm::vec2{glm::rotate(glm::mat4{1.0f}, rotation_,
                                          glm::vec3{0.0f, 0.0f, 1.0f}) *
                              glm::vec4{offset, 0.0f, 0.0f}};
    if (!game_core_->IsBlockedByObstacles(new_position)) {
      game_core_->PushEventMoveUnit(id_, new_position);
    }
    float rotation_offset = 0.0f;
    if (input_data.key_down[GLFW_KEY_A]) {
      rotation_offset += 1.0f;
    }
    if (input_data.key_down[GLFW_KEY_D]) {
      rotation_offset -= 1.0f;
    }
    rotation_offset *= kSecondPerTick * rotate_angular_speed * GetSpeedScale();
    game_core_->PushEventRotateUnit(id_, rotation_ + rotation_offset);
  }
}

void Edgar::TurretRotate() {
  auto player = game_core_->GetPlayer(player_id_);
  if (player) {
    auto &input_data = player->GetInputData();
    auto diff = input_data.mouse_cursor_position - position_;
    if (glm::length(diff) < 1e-4) {
      turret_rotation_ = rotation_;
    } else {
      turret_rotation_ = std::atan2(diff.y, diff.x) - glm::radians(90.0f);
    }
  }
}

void Edgar::Fire() {
  if (fire_count_down_ == 0) {
    auto player = game_core_->GetPlayer(player_id_);
    if (player) {
      auto &input_data = player->GetInputData();
      if (input_data.mouse_button_down[GLFW_MOUSE_BUTTON_LEFT]) {

        auto base_velocity = Rotate(glm::vec2{0.0f, 20.0f}, turret_rotation_);

        auto base_position = position_ + Rotate({0.0f, 1.2f}, turret_rotation_);

        std::vector<float> angles = {-30.0f, 0.0f, 30.0f};
        for (int i = 0; i < 3; ++i) {
          float angle_offset = angles[i] * glm::radians(1.0f);
          for (int j = 0; j < 4; ++j) {
            float bullet_angle = turret_rotation_ + angle_offset + j * glm::radians(30.0f);
            auto bullet_position = base_position;
            auto bullet_velocity = Rotate(base_velocity, bullet_angle - turret_rotation_);
            GenerateBullet<bullet::CannonBall>(
                bullet_position, bullet_angle, GetDamageScale(), bullet_velocity);
          }
          if (i < 2) {
            fire_count_down_ = static_cast<uint32_t>(0.3f * kTickPerSecond);
            return;
          }
        }
        fire_count_down_ = kTickPerSecond;
      }
    }
  } else {
    fire_count_down_--;
  }
}

void Edgar::HandleMarker() {
        auto player = game_core_->GetPlayer(player_id_);
        if (player) {
            auto &input_data = player->GetInputData();
            if (input_data.key_down[GLFW_KEY_J]) {
                marker_position_ = position_;
                has_marker_ = true;
            }
            if (input_data.key_down[GLFW_KEY_K] && has_marker_) {
                game_core_->PushEventMoveUnit(id_, marker_position_);
                has_marker_ = false;
            }
        }
    }

void Edgar::HandleBoost() {
    if (boost_cooldown_ > 0) {
        boost_cooldown_--;
        return;
    }

    auto player = game_core_->GetPlayer(player_id_);
    if (player) {
        auto &input_data = player->GetInputData();
        if (input_data.key_down[GLFW_KEY_F]) {
            glm::vec2 boost_offset{0.0f, 5.0f};  // 猛冲距离
            glm::vec2 new_position = position_ + boost_offset;

            if (!game_core_->IsBlockedByObstacles(new_position)) {
                game_core_->PushEventMoveUnit(id_, new_position);
                boost_cooldown_ = 10;  // 设置猛冲冷却时间，例如10个tick
            }
        }
    }
}
bool Edgar::IsHit(glm::vec2 position) const {
  position = WorldToLocal(position);
  // Simplified hit detection for a hexagon
  return glm::length(position) < 0.8f;
}

const char *Edgar::UnitName() const {
  return "506";
}

const char *Edgar::Author() const {
  return "suyc24";
}
}  // namespace battle_game::unit