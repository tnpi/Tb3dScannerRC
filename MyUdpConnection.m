//
//  MyUdpConnection.m
//  udp_test
//
//  Created by ogawakeisuke on 2016/01/19.
//  Copyright (c) 2016年 ogawakeisuke. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "MyUdpConnection.h"


@implementation MyUdpConnection

// 初期化
- (id)initWithDelegate:(id)_delegate portNum:(int)_port {
    delegate = _delegate;
    port = _port;
    
    return self;
}

// 受信開始
- (void)bind {
    NSThread *th = [[NSThread alloc]initWithTarget:self selector:@selector(bindThread) object:nil];
    [th start];
}

// 受信用スレッド
- (void)bindThread {

    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_port = htons(port); //適当なポートで待機
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    
    char buf[100000]; //100KBまで対応
    while (1) {
        @autoreleasepool {
            //ここでデータを受信するまでブロックされる
            long size = recv(sock, buf, sizeof(buf), 0);
        
            //NSDataに変換し、delegateに通知
            NSData *data = [NSData dataWithBytes:buf length:size];
            [delegate performSelectorOnMainThread:@selector(receiveUdpData:) withObject:data waitUntilDone:YES];
        }
    }
}
@end