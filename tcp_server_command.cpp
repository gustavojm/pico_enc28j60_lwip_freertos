// #include <stdlib.h>
#include "FreeRTOS.h"
#include <cctype>
#include <memory>
#include <stdio.h>
#include <string.h>

#include "tcp_server_command.h"

#include <lwip/sockets.h>


namespace json = ArduinoJson;

json::JsonDocument tcp_server_command::led_switch_cmd(json::JsonObject const pars) {
    json::JsonDocument res;
    res["STATUS"] = "ON";
    return res;
}

// @formatter:off
const tcp_server_command::cmd_entry tcp_server_command::cmds_table[] = {
    {
        "LED_SWITCH",
        &tcp_server_command::led_switch_cmd,
    },
};
// @formatter:on

/**
 * @brief 	searchs for a matching command name in cmds_table[], passing the
 * parameters as a JSON object for the called function to parse them.
 * @param 	*cmd 	:name of the command to execute
 * @param   *pars   :JSON object containing the passed parameters to the called
 * function
 */
json::JsonDocument tcp_server_command::cmd_execute(char const *cmd, json::JsonObject const pars) {
    json::JsonDocument res;
    for (unsigned int i = 0; i < (sizeof(cmds_table) / sizeof(cmds_table[0])); i++) {
        if (!strcmp(cmd, cmds_table[i].cmd_name)) {
            // return cmds_table[i].cmd_function(pars);
            return (this->*(cmds_table[i].cmd_function))(pars);
        }
        res.set("Command Response");
        return res;
    }
    printf("No matching command found");
    res.set("UNKNOWN COMMAND");
    return res;
};

bool tcp_server_command::reply_fn(int conn_sock) {
    char rx_buff[128];
    char *tx_buff;
    int done = 0;
    while (done < sizeof(rx_buff)) {
        int done_now = recv(conn_sock, rx_buff + done, sizeof(rx_buff) - done, 0);
        if (done_now <= 0)
            return -1;

        done += done_now;
        char *end = strnstr(rx_buff, "\0", done);
        if (!end)
            continue;

        auto rx_JSON_value = json::JsonDocument();
        //printf("%s", rx_buff);
        json::DeserializationError error = json::deserializeJson(rx_JSON_value, rx_buff);

        auto tx_JSON_value = json::JsonDocument();
        tx_buff = nullptr;
        int buff_len = 0;

        if (error) {
            printf("Error json parse. %s", error.c_str());
        } else {
            for (json::JsonVariant command : rx_JSON_value.as<json::JsonArray>()) {
                char const *command_name = command["cmd"];
                printf("Command Found: %s", command_name);
                auto pars = command["pars"];
                auto ans = cmd_execute(command_name, pars);
                tx_JSON_value[command_name] = ans;
            }

            buff_len = json::measureJson(tx_JSON_value); /* returns 0 on fail */
            buff_len++;
            tx_buff = new char[buff_len];
            if (!(*tx_buff)) {
                printf("Out Of Memory");
                buff_len = 0;
            } else {
                json::serializeJson(tx_JSON_value, tx_buff, buff_len);
                printf("%s", tx_buff);
                send_message(conn_sock, tx_buff);
            }
        }

        break;
    }

    return 0;
}
