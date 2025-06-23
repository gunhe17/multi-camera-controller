#pragma once

#include <iostream>
#include <sstream>
#include <windows.h>


/**
 * Config
 */

// Parent

class ConfigError : public std::runtime_error {
public:
    ConfigError(
        const std::string& type, 
        const std::string& msg, 
        int code
    ): 
        std::runtime_error("[ Config Error] " + type + ": " + msg + " (code: " + std::to_string(code) + ")"),
        type_(type), 
        message_(msg), 
        code_(code) 
        {}

    const std::string& getType() const noexcept { return type_; }
    const std::string& getMessage() const noexcept { return message_; }
    int getCode() const noexcept { return code_; }

private:
    std::string type_;
    std::string message_;
    int code_;
};


// Child

class ConfigInputError: public ConfigError {
public:
    ConfigInputError(const std::string& msg, int code = 400)
        : ConfigError("InputError", msg, code) {}
};

class ConfigCameraError : public ConfigError {
public:
    ConfigCameraError(const std::string& msg, int code = 400)
        : ConfigError("CameraError", msg, code) {}
};

class ConfigRecordingError : public ConfigError {
public:
    ConfigRecordingError(const std::string& msg, int code = 400)
        : ConfigError("RecordingError", msg, code) {}
};

class ConfigExternalError : public ConfigError {
public:
    ConfigExternalError(const std::string& msg, int code = 400)
        : ConfigError("ExternalError", msg, code) {}
};

class ConfigWarmupError : public ConfigError {
public:
    ConfigWarmupError(const std::string& msg, int code = 400)
        : ConfigError("WarmupError", msg, code) {}
};

class ConfigOutputError : public ConfigError {
public:
    ConfigOutputError(const std::string& msg, int code = 400)
        : ConfigError("OutputError", msg, code) {}
};