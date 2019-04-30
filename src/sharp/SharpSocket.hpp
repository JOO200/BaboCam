//
// Created by johannes on 4/24/19.
//

#ifndef BABOCAM_SHARPSOCKET_HPP
#define BABOCAM_SHARPSOCKET_HPP

#include "../interfaces/IRunnable.hpp"
#include "../struct/Context.hpp"

class SharpSocket : public IRunnable {
public:
    explicit SharpSocket(Context * context, std::string target = "192.168.7.2", short port = 25560);

protected:
    void run() override;

private:
    Context * m_context;
    bool validate_json(std::string & incoming);
    std::string m_target;
    short m_port;
};


#endif //BABOCAM_SHARPSOCKET_HPP
