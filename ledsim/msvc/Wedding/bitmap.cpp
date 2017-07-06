#include "bitmap.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

bitmap::bitmap()
{
}


bitmap::~bitmap()
{
}

bool bitmap::read_bmp(char *filename)
{
	FILE *fp;
	unsigned char header_buf[54];	//�w�b�_������荞��
	int real_width;
	unsigned int color;
	unsigned char *bmp_line_data;

	/* �t�@�C���ǂݍ��� */
	if ((fp = fopen(filename, "rb")) == NULL){
		fprintf(stderr, "Error: %s could not read.", filename);
		return false;
	}

	fread(header_buf, sizeof(unsigned char), sizeof(header_buf), fp); //�w�b�_�����S�Ă���荞��

	//�ŏ���2�o�C�g��BM(Bitmap�t�@�C���̈�)�ł��邩
	if (strncmp((const char *)header_buf, "BM", 2)){
		fprintf(stderr, "Error: %s is not Bitmap file.", filename);
		return false;
	}
	memcpy(&width, header_buf + 18, sizeof(width)); //�摜�̌����ڏ�̕����擾
	memcpy(&height, header_buf + 22, sizeof(height)); //�摜�̍������擾
	memcpy(&color, header_buf + 28, sizeof(unsigned int)); //��bit��Bitmap�ł��邩���擾
	//24bit�Ŗ�����ΏI��
	if (color != 24){
		fprintf(stderr, "Error: %s is not 24bit color image", filename);
		return false;
	}

	//RGB���͉摜��1�s����4byte�̔{���Ŗ�����΂Ȃ�Ȃ����߂���ɍ��킹�Ă���
	real_width = width * 3 + width % 4;

	//�摜��1�s����RGB��������Ă��邽�߂̃o�b�t�@�𓮓I�Ɏ擾
	if ((bmp_line_data = (unsigned char *)malloc(sizeof(char) * real_width)) == NULL){
		fprintf(stderr, "Error: Allocation error.\n");
		return false;
	}

	if ((Rdata = (unsigned char*)malloc(sizeof(char) * width * height)) == NULL){
		fprintf(stderr, "Allocation error\n");
		free(Rdata);
		return false;
	}
	if ((Gdata = (unsigned char*)malloc(sizeof(char) * width * height)) == NULL){
		fprintf(stderr, "Allocation error\n");
		free(Gdata);
		return false;
	}
	if ((Bdata= (unsigned char*)malloc(sizeof(char) * width * height)) == NULL){
		fprintf(stderr, "Allocation error\n");
		free(Bdata);
		return false;
	}

	//Bitmap�t�@�C����RGB���͍�������E�ցA�������ɕ���ł���
	for (int i = 0; i < height; i++){
		fread(bmp_line_data, 1, real_width, fp);
		for (int j = 0; j < width; j++){
			Bdata[(height - i - 1)*width + j] = bmp_line_data[j * 3];
			Gdata[(height - i - 1)*width + j] = bmp_line_data[j * 3 + 1];
			Rdata[(height - i - 1)*width + j] = bmp_line_data[j * 3 + 2];
		}
	}

	return true;
}