#include <iostream>
#include <curl/curl.h>
#include <string>

size_t write_callback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

int main() {
    CURL* curl;
    CURLcode res;
    std::string response_data;

    std::string json_data = R"({
        "power": 0,
        "auxpart_pwr": 0,
        "card_pwr": 0,
        "mascot_pwr": 0,
        "ps_card_pwr": 0,
        "club_index": 0,
        "shot_index": 0,
        "power_shot_index": 0,
        "distance": 200,
        "height": 4,
        "wind": 5,
        "degree": 47,
        "ground": 100,
        "spin": 5,
        "curve": 5,
        "slope_break": 0
    })";

    std::cout << "Enviando JSON:\n" << json_data << "\n\n";

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:3000/calcular");
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_data.size());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            std::cerr << "Erro na requisição: " << curl_easy_strerror(res) << std::endl;
        }
        else {
            std::cout << "Resposta do servidor (formato amigável):\n";

            std::string formatted = response_data;
            for (auto& ch : formatted) {
                if (ch == ',') {
                    ch = '\n';
                }
            }

            std::cout << formatted << std::endl;
        }
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }

    curl_global_cleanup();
    return 0;
}
