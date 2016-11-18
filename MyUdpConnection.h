//
//  MyUdpConnection.h
//  udp_test
//
//  Created by ogawakeisuke on 2016/01/19.
//  Copyright (c) 2016年 ogawakeisuke. All rights reserved.
//


#import <netinet/in.h>
#import <Foundation/Foundation.h>

@interface MyUdpConnection : NSObject {
    id delegate;
    int port;
}
- (id)initWithDelegate:(id)_delegate portNum:(int)_port; //receiveData:(NSData*)data を実装すること
- (void)bind;
- (void)bindThread;
@end
