// %% license-end-token %%

//
// This file was generated by Coral RPC.
//
// WARNING: Auto-generated code. Do not modify.
//

#ifndef SCANNERSERVICE_SETMODE_ACTION_H
#define SCANNERSERVICE_SETMODE_ACTION_H

#include "RpcServerResource.h"
#include "ScannerRpc.pb.h"

namespace ScannerRpc {

class ScannerServiceSetModeAction : public coral::rpc::RpcServiceAction {
public:

   ScannerServiceSetModeAction();

   void operator() (
      const std::string&        request,
      std::string&              response,
      coral::rpc::RpcException& e );

protected:

   virtual void SetMode(
      const Request& request,
      Response& response,
      coral::rpc::RpcException& e );

};

}  // End namespace ScannerRpc

#endif // SCANNERSERVICE_SETMODE_ACTION_H
