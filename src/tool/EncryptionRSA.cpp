#include <codecvt>
#include <locale>
#include <stdio.h>
#include <cstring>
#include "EncryptionRSA.h"

using namespace Encrypt;

inline char* long2charArr(long num)
{
	//std::cout<<"long2charArr"<<std::endl;
	char* arr=new char[8];
	//char* arr;
	int wei = 56;
	for(int i=7;i>=0;i--)
	{
		long temp = num<<wei;
		arr[i]=temp>>56;
		wei-=8;
	}
	return arr;
}
inline long long extend_gcd(long long a, long long b, long long &x, long long &y) 
{
	if (a == 0 && b == 0) return -1;//无最大公约数
	if(b==0) {
		x=1;y=0;return a;
	}
	long long d=extend_gcd(b,a%b,y,x);
	y-=a/b*x;
	return d;
}
	//ax = 1(mod n) 求X
long long mod_reverse(long long a, long long n) 
{
	long long x, y;
	long long d = extend_gcd(a, n, x, y);
	if (d == 1)
		return (x%n + n) % n;
	else return -1;
}
 
EncryptionRSA::EncryptionRSA() {
	//ProduceKeys();
}
void EncryptionRSA::Ecrept(const string &plain_file_in,const string &key_name_file)
{
	ProduceKeys(key_name_file);
	string ecrept_file_out = plain_file_in +".ecrept";
	std::ifstream fin(plain_file_in);
	std::ofstream fout(ecrept_file_out, std::ofstream::app);
	if (!fin.is_open()){
		std::cout << "open file failed" << std::endl;
		return;
	}
	
	// fin.seekg(0, ios::end); //设置文件指针到文件流的尾部
	// std::streampos ps = fin.tellg(); //读取文件指针的位置

	const int NUM = 256;
	char buf[NUM];
	long buf_out[NUM];
	int cur_num;
	//std::string in_buf;
	while (!fin.eof()){
		fin.read(buf, NUM);
		
		cur_num = fin.gcount();
		for (int i = 0; i < cur_num; ++i){
			//buf_out[i] = Ecrept((long)buf[i], public_key, share_key);
			buf_out[i] = Ecrept((long)buf[i], _key.public_key, _key.share_key);
			//in_buf.push_back(buf_out[i]);
		}
		fout.write((char*)buf_out, cur_num * sizeof(long));
	}
	fin.close();
	remove(plain_file_in.c_str());
	fout.close();
}
void EncryptionRSA::DEcrept(const string &plain_file_in,const string &key_name_file){
	string tmp = ".ecrept";
	int tmpLen = tmp.length();
	int fileLen =plain_file_in.length();
	int Len_=fileLen-tmpLen;
	string ecrept_file_out = plain_file_in.substr(0,Len_);
	//std::string key_name = "key.txt";
	std::ifstream key_file(key_name_file);
	if(!key_file.is_open()){
		std::cout << "open key file failed" << std::endl;
		return;
	}
	char sh_key[8];
	std::string key_s;
	long private_key;
	long share_key;
	int i_count=0;
	while(std::getline(key_file, key_s))
	{
		if(i_count ==0)
		{
			share_key = std::atoi(key_s.c_str());
			key_s.clear();
		}
		if(i_count ==1)
		{
			private_key = std::atoi(key_s.c_str());
			key_s.clear();
		}
		i_count++;
	}
	key_file.close();
	//remove(key_name_file.c_str());

	std::ifstream fin(plain_file_in);
	std::ofstream fout(ecrept_file_out, std::ofstream::app);
	if (!fin.is_open()){
		std::cout << "open file failed" << std::endl;
		return;
	}
	
	const int NUM = 256;
	long buf[NUM];
	char buf_out[NUM];
	int cur_num;
	while (!fin.eof()){
		fin.read((char*)buf, NUM * sizeof(long));
		//当前所读取的字节数
		cur_num = fin.gcount();
		cur_num /= sizeof(long);
		for (int i = 0; i < cur_num; ++i){
			//std::cout<<buf[i]<<"   ";
			buf_out[i] = (char)Ecrept((long)buf[i], private_key, share_key);
			///std::cout<<buf_out[i];
		}
		//std::cout<<""<<std::endl;
		fout.write(buf_out, cur_num);
	}
	fin.close();
	//remove(plain_file_in.c_str());
	fout.close();
}

std::vector<long> EncryptionRSA::Ecrept(std::string& str_in, long public_key, long share_key) {
	std::vector<long> vecout;
	for (const auto& e : str_in){
		vecout.push_back(Ecrept(e, public_key, share_key));
	}
	return vecout;
}
std::string EncryptionRSA::DEcrept(std::vector<long>& ecrept_str, long private_key, long share_key) {
	std::string strout;
	for (const auto& e : ecrept_str){
		strout.push_back((char)Ecrept(e, private_key, share_key));
	}
	return strout;
}

void EncryptionRSA::PrintInfo(std::vector<long>& ecrept_str) {
	for (const auto& e : ecrept_str){
		std::cout << e << " ";
	}
	std::cout << std::endl;
}
 
long EncryptionRSA::Ecrept(long msg, long key, long share_key){
	//std::cout<<"input msg :"<<msg<<std::endl;
	long msg_out = 1;
	long a = msg;
	long b = key;
	int c = share_key;
	while (b){
		if (b & 1){
			
			msg_out = (msg_out * a) % c;
		}
		b >>= 1;
		a = (a * a) % c;
	}
	//std::cout<<"output msg :"<<msg_out<<std::endl;
	return msg_out;
}
 
//产生素数，随机产生两个素数
long EncryptionRSA::ProducePrime()
{
	srand(time(nullptr));
	long prime = 0;
	while (1){
		prime = rand() % 50 + 2;
		if (IsPrime(prime))
			break;
	}
	return prime;
}
 
//判断一个数是否是素数
bool EncryptionRSA::IsPrime(long prime) {
	if (prime < 2)
		return false;
	for (int i = 2; i < sqrt(prime); ++i){
		if (prime % i == 0)
			return false;
	}
	return true;
}
 
//产生所有的key值
void EncryptionRSA::ProduceKeys(const string &key_file) {
	//选择两个不相等的素数
	// long prime1 = ProducePrime();
	// long prime2 = ProducePrime();
	long prime1 = prime[rand()%90];
	long prime2 = prime[rand()%90];
	while (prime1 == prime2)
		prime2=prime[rand()%90];
		//prime2 = ProducePrime();
	_key.share_key = ProduceShareKey(prime1, prime2);
	long orla = ProduceOrla(prime1, prime2);
	//_key.public_key = ProducePublicKey(orla);
	_key.public_key = 65537;
	_key.private_key = ProducePrivateKey(_key.public_key, orla);
	std::ofstream file_in(key_file, std::ofstream::app);
	if(!file_in.is_open())
	{
		std::cout<<"open key txt fail"<<std::endl;
	}
	file_in<<_key.share_key<<"\n"<<_key.private_key<<std::endl;
	file_in.close();
}
 
//求share_kay
long EncryptionRSA::ProduceShareKey(long prime1, long prime2) {
	return prime1 * prime2;
}
 
//根据欧拉函数求乘积
long EncryptionRSA::ProduceOrla(long prime1, long prime2) {
	return (prime1 - 1) * (prime2 - 1);
}
 
//求public_key，随机选择一个数， 1 < public_key < orla，public_key,oala互质
long EncryptionRSA::ProducePublicKey(long orla) {
	long public_key;
	srand(time(nullptr));
	while (1){
		public_key = rand() % orla;
		if (public_key > 1 && ProduceGcd(public_key, orla) == 1)
			break;
	}
	return public_key;
}
 
//判断两个数之间的最大公约是否为1
long EncryptionRSA::ProduceGcd(long public_key, long orla) {
	long residual;
	while (residual = public_key % orla){
		public_key = orla;
		orla = residual;
	}
	return orla;
}
 
//求private_key
long EncryptionRSA::ProducePrivateKey(long public_key, long orla) {
	//(public_key * private_key) % orla == 1
	long private_key = orla / public_key;
	while (1){
		if ((public_key * private_key) % orla == 1)
			break;
		++private_key;
	}
	return private_key;
}


