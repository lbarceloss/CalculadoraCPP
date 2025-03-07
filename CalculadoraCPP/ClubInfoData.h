#pragma once
#include <unordered_map>
#include <string>
#include "ClubInfo.h"

// Enum para os tipos de tacos específicos
enum class ClubInfoEnum {
    _1W, _2W, _3W, _2I, _3I, _4I, _5I, _6I, _7I, _8I, _9I,
    PW, SW, PT1, PT2
};

// Classe que armazena os dados dos tacos
class ClubInfoData {
public:
    // Mapa estático com os dados dos tacos
    static std::unordered_map<std::string, ClubInfo> CLUB_INFO;

    // Método para inicializar os dados (substitui a inicialização estática do C#)
    static void Init() {
        CLUB_INFO["_1W"] = ClubInfo(ClubType::WOOD, 0.55, 1.61, 236.0, 10.0, 230.0);
        CLUB_INFO["_2W"] = ClubInfo(ClubType::WOOD, 0.50, 1.41, 204.0, 13.0, 210.0);
        CLUB_INFO["_3W"] = ClubInfo(ClubType::WOOD, 0.45, 1.26, 176.0, 16.0, 190.0);
        CLUB_INFO["_2I"] = ClubInfo(ClubType::IRON, 0.45, 1.07, 161.0, 20.0, 180.0);
        CLUB_INFO["_3I"] = ClubInfo(ClubType::IRON, 0.45, 0.95, 149.0, 24.0, 170.0);
        CLUB_INFO["_4I"] = ClubInfo(ClubType::IRON, 0.45, 0.83, 139.0, 28.0, 160.0);
        CLUB_INFO["_5I"] = ClubInfo(ClubType::IRON, 0.45, 0.73, 131.0, 32.0, 150.0);
        CLUB_INFO["_6I"] = ClubInfo(ClubType::IRON, 0.41, 0.67, 124.0, 36.0, 140.0);
        CLUB_INFO["_7I"] = ClubInfo(ClubType::IRON, 0.36, 0.61, 118.0, 40.0, 130.0);
        CLUB_INFO["_8I"] = ClubInfo(ClubType::IRON, 0.30, 0.57, 114.0, 44.0, 120.0);
        CLUB_INFO["_9I"] = ClubInfo(ClubType::IRON, 0.25, 0.53, 110.0, 48.0, 110.0);
        CLUB_INFO["PW"] = ClubInfo(ClubType::PW, 0.18, 0.49, 107.0, 52.0, 100.0);
        CLUB_INFO["SW"] = ClubInfo(ClubType::PW, 0.17, 0.42, 93.0, 56.0, 80.0);
        CLUB_INFO["PT1"] = ClubInfo(ClubType::PT, 0.00, 0.00, 30.0, 0.00, 20.0);
        CLUB_INFO["PT2"] = ClubInfo(ClubType::PT, 0.00, 0.00, 21.0, 0.00, 10.0);
    }
};

// Inicialização do mapa estático
std::unordered_map<std::string, ClubInfo> ClubInfoData::CLUB_INFO;
