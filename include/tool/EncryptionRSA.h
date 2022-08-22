#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <fstream>
#include <math.h>

namespace Encrypt
{
using namespace std;

struct Key{
	//公钥(public_key, share_key)
	long long  share_key;
	long long  public_key;
	//私钥(private_key, share_key)
	long long  private_key;
};
 
class EncryptionRSA{
public:
	EncryptionRSA();
	Key GetKey() {
		return _key;
	}
	//给文件进行加密
	void Ecrept(const string &plain_file_in,const string &key_name_file);
	void DEcrept(const string &plain_file_in,const string &key_name_file);
 
	//对字符串进行加密
	std::vector<long> Ecrept(std::string& str_in, long public_key, long share_key);
	std::string DEcrept(std::vector<long>& ecrept_str, long private_key, long share_key);
 
	//打印加密之后的信息
	void PrintInfo(std::vector<long>& ecrept_str);
private:
	//产生素数
	long ProducePrime();
	//判断一个数是否是素数
	bool IsPrime(long prime);
	//产生所有的key值
	void ProduceKeys(const string &key_name_file);
	//求share_kay
	long ProduceShareKey(long prime1, long prime2);
	//根据欧拉函数求乘积
	long ProduceOrla(long prime1, long prime2);
	//求public_key
	long ProducePublicKey(long orla);
	//判断两个数之间的最大公约是否为1
	long ProduceGcd(long public_key, long orla);
	//求private_key
	long ProducePrivateKey(long public_key, long orla);
	//加密单个信息
	long Ecrept(long msg, long key, long share_key);
private:
	Key _key;
	int prime[90] = { 401, 409, 419, 421, 431, 433, 439,
		443, 449, 457, 461, 463, 467, 479, 487, 491, 499,
		503, 509, 521, 523, 541, 547, 557, 563, 569, 571,
		577, 587, 593, 599, 601, 607, 613, 617, 619, 631,
		641, 643, 647, 653, 659, 661, 673, 677, 683, 691,
		701, 709, 719, 727, 733, 739, 743, 751, 757, 761,
		769, 773, 787, 797, 809, 811, 821, 823, 827, 829,
		839, 853, 857, 859, 863, 877, 881, 883, 887, 907,
		911, 919, 929, 937, 941, 947, 953, 967, 971, 977,
		983, 991, 997 };
};

}
