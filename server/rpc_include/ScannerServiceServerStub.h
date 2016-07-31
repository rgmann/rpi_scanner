// %% license-end-token %%

//
// This file was generated by Coral RPC.
//
// WARNING: Auto-generated code. Do not modify.
//

#ifndef SCANNERSERVICE_SERVER_STUB_H
#define SCANNERSERVICE_SERVER_STUB_H

#include "RpcServerResource.h"

#include "ScannerServiceSetModeAction.h"

namespace ScannerRpc {

class ScannerServiceServerStub : public coral::rpc::RpcServerResource {
public:

   explicit ScannerServiceServerStub();
   ~ScannerServiceServerStub();

   virtual void registerActions();


private:

   ScannerServiceSetModeAction default_setmode_action_;

}; // End ScannerServiceServerStub

}  // End namespace ScannerRpc


#endif // SCANNERSERVICE_SERVER_STUB_H