#include "animation.h"
#include "LED.h"
#include "draw.h"
#include <math.h>

static double image_scale = (double)size / (double)LED_LENGTH / SCALE_FACTOR;

static const float size_2 = size / 2;
static const int led_length_2 = LED_LENGTH / 2;

void polar_image(struct led *buffer, struct led image[size][size], double angle){
    int i;
    int x, y;
    limit_angle(&angle);
    
    static float rad, r, c, s, co, so, scale, xi, yi;
    
    rad = angle * 3.1415 / 180.0;
    c = cos(rad);
    s = sin(rad);
    
//    angle = angle + OPP_ANGLE_CORRECTION + 180.0;
//    limit_angle(&angle);    
//    co = cos(rad);
//    so = sin(rad);
    
    co = -c;
    so = -s;
    
//    xi = image_scale*c;
//    yi = image_scale*s;
//    x = -0.75*xi + size_2;
//    y = 0.75*yi + size_2;    
    for(i = 0; i < led_length_2; i++) {
        r = (double)i + 0.75;
        x = -image_scale*r*c + size / 2;
        y = image_scale*r*s + size / 2;
        if(x >= size || x < 0 || y >= size || y < 0) 
            continue;
        buffer[led_length_2 - 1 - i] = image[y][x];
    }
    
//    xi = image_scale*co;
//    yi = image_scale*so;
//    x = -0.25*xi + size_2;
//    y = 0.25*yi + size_2;   
    for(i = 0; i < led_length_2; i++) {
        r = (double)i + 0.25;
        x = -image_scale*r*co + size / 2;
        y = image_scale*r*so + size / 2;
        if(x >= size || x < 0 || y >= size || y < 0)
            continue;
        buffer[led_length_2 + i] = image[y][x];
    }
}

void scaleBrightness_image(struct led image[size][size], float min_scale) {
    int i, j;
    float r, scale;
    static float center2 = size / 2 * size / 2;
    
    for(i = 0; i < size_2; i++) {
        for(j = 0; j < size_2; j++) {
            r = sqrt(pow(i - size_2, 2) + pow(j - size_2, 2));              
            scale = r/size_2 * (1.0 - min_scale) + min_scale;
            
//            r = pow(i - size_2, 2) + pow(j - size_2, 2);              
//            scale = r/center2 * (1.0 - min_scale) + min_scale;
            
            image[i][j].red   *= scale;
            image[i][j].green *= scale;
            image[i][j].blue  *= scale;
            
            image[i][size-j].red   *= scale;
            image[i][size-j].green *= scale;
            image[i][size-j].blue  *= scale;
            
            image[size-i][j].red   *= scale;
            image[size-i][j].green *= scale;
            image[size-i][j].blue  *= scale;
            
            image[size-i][size-j].red   *= scale;
            image[size-i][size-j].green *= scale;
            image[size-i][size-j].blue  *= scale;
        }
    }
}

struct dict {
    int data;
    int prev;
    int len;
};

static struct dict lzw_dict[4096];
static struct led GCT_table[256];
static struct led local_table[256];

static unsigned int width, height;
static unsigned int GCT_size, packet_fields, background_index;
static int clear_code, stop_code, reset_code_length;

static unsigned long frame_addr[100];
static int trnsp_index[100];

unsigned char gif_init() {
    unsigned long index, i, dict_i, image_index = 0;
    int len;
    int bit_index;

    unsigned char left_pos, top_pos, sub_width, sub_height;

    int code, prev = -1, ptr;
    int match_len, code_len;

    int frame_count = 0;

    //Read header
    width = gif[6] | (gif[7] << 8);
    height = gif[8] | (gif[9] << 8);

    packet_fields = gif[10];
    GCT_size = 1 << ((packet_fields & 0b111) + 1);
    
    background_index = gif[11];

    //Read GCT table
    for(i = 0, index = 13; i/3 < GCT_size; i += 3) {
        GCT_table[i/3].red = gif[index + i];
        GCT_table[i/3].green = gif[index + i + 1];
        GCT_table[i/3].blue = gif[index + i + 2];
    }
    index += i;

    //Skip to image data
    while(gif[index] != 0x2C) {
        if(gif[index] == 0x21 && gif[index+1] == 0xF9) {
            if(gif[index+3] & 1 == 1) {
                trnsp_index[frame_count] = gif[index+6];
            }
            else {
                trnsp_index[frame_count] = -1;
            }
        }
        index+=2;
        while(gif[index] != 0) {
            len = gif[index++];
            index += len;
        }
        index++;
    }
    index++;
    frame_addr[frame_count++] = index;

    if(gif[index+8] & 0x80) {
        unsigned int local_size = 1 << ((gif[index+8] & 0b111) + 1);
        index += local_size * 3;
    }
    index += 10;

    while(1) {
        len = gif[index];
        index += len + 1;

        if(gif[index] == 0) {
            index++;
            if(gif[index] == 0x3B) {
                break;
            }
            while(gif[index] != 0x2C) {
                if(gif[index] == 0x21 && gif[index+1] == 0xF9) {
                    if(gif[index+3] & 1 == 1) {
                        trnsp_index[frame_count] = gif[index+6];
                    }
                    else {
                        trnsp_index[frame_count] = -1;
                    }
                }
                index+=2;
                while(gif[index] != 0) {
                    len = gif[index++];
                    index += len;
                }
                index++;
            }
            index++;
            frame_addr[frame_count++] = index;

            if(gif[index+8] & 0x80) {
                unsigned int local_size = 1 << ((gif[index+8] & 0b111) + 1);
                index += local_size * 3;
            }
            index += 10;
        }
    }

    return frame_count;
}

unsigned char gif_get_frame(struct led image[size][size], unsigned char frame_no) {
    unsigned long int index, i, j, dict_i, image_index = 0;
    unsigned int pixel_index;
    int len, temp_len;
    int bit_index;
    unsigned int local_size = 0;

    unsigned int left_pos, top_pos, sub_width, sub_height, t;

    int code, prev = -1, ptr;
    int match_len, code_len;

    index = frame_addr[frame_no];

    //Read image descriptor
    left_pos = gif[index] | (gif[index+1] << 8);
    index += 2;
    top_pos = gif[index] | (gif[index+1] << 8);
    index += 2;

    sub_width = gif[index] | (gif[index+1] << 8);
    index += 2;
    sub_height = gif[index] | (gif[index+1] << 8);
    index += 2;

    if(gif[index] & 0x80) {
        local_size = 1 << ((gif[index] & 0b111) + 1);
        index++;
        for(i = 0; i/3 < local_size; i += 3) {
            local_table[i/3].red = gif[index + i];
            local_table[i/3].green = gif[index + i + 1];
            local_table[i/3].blue = gif[index + i + 2];
        }
        index += (i - 1);
    }
    
//    if(frame_no == 0) {
//        for(i = 0; i < size; i++){
//            for(j = 0; j < size; j++){
//                if(local_size) {
//                    image[i][j] =  local_table[background_index];
//                } else {
//                    image[i][j] =  GCT_table[background_index];
//                }
//            }
//        }
//    }

    index++;

    code_len = gif[index++];

    clear_code = 1 << code_len;
    stop_code = clear_code + 1;
    reset_code_length = code_len;

    unsigned int mask = 0x01;
    int data_bit;

    temp_len = 0;

    while(1) {
        if(temp_len == 0) {
            len = gif[index++];
            temp_len = len;
        }

        if(len == 0x00) { // End of
            return 0;
        }

        for(bit_index = 0, code = 0; bit_index < (code_len + 1); bit_index++ ) {
            data_bit = (gif[index] & mask ) ? 1 : 0;
            mask <<= 1;

            if(mask == 0x100) {
                mask = 0x01;
                index++;
                temp_len--;
                if(temp_len == 0) {
                    len = gif[index++];
                    temp_len = len;
                    if(len == 0x00) { // End of stream
                        return 0;
                    }
                }
            }
            code = code | (data_bit << bit_index);
        }

        if(code == clear_code) {
            code_len = reset_code_length;

            for(dict_i = 0; dict_i < 4096; dict_i++) {
                lzw_dict[dict_i].data = -1;
                lzw_dict[dict_i].prev = -1;
                lzw_dict[dict_i].len = -1;
            }

            for(dict_i = 0; dict_i < (1 << code_len); dict_i++ ) {
                lzw_dict[dict_i].data = dict_i;
                lzw_dict[dict_i].prev = -1;
                lzw_dict[dict_i].len = 1;
            }
            dict_i += 2;
            prev = -1;
            continue;
        }
        else if(code == stop_code) {
            if(len > 1) {
                return -1;
            }
            break;
        }
        if((prev > -1) && (code_len < 12) && dict_i < 4095) {
            if(code == dict_i) {
                ptr = prev;

                while(lzw_dict[ptr].prev != -1) {
                    ptr = lzw_dict[ptr].prev;
                }
                lzw_dict[dict_i].data = lzw_dict[ptr].data;
            }
            else {
                ptr = code;
                while(lzw_dict[ptr].prev != -1) {
                    ptr = lzw_dict[ptr].prev;
                }
                lzw_dict[dict_i].data = lzw_dict[ptr].data;
            }

            lzw_dict[dict_i].prev = prev;
            lzw_dict[dict_i].len = lzw_dict[prev].len + 1;
            dict_i++;

            if((dict_i == ( 1 << ( code_len + 1))) && (code_len < 11)) {
              code_len++;
            }
        }
        prev = code;
        match_len = lzw_dict[code].len;
        while(code != -1) {
            pixel_index = image_index + lzw_dict[code].len - 1;

            if((lzw_dict[code].data != trnsp_index[frame_no]) || trnsp_index[frame_no] < 0) {
                if(lzw_dict[code].data > local_size && local_size) {
                }
                if(local_size) {
                    image[pixel_index/sub_width + top_pos][pixel_index%sub_width + left_pos] = local_table[lzw_dict[code].data];
                } else {
                    image[pixel_index/sub_width + top_pos][pixel_index%sub_width + left_pos] = GCT_table[lzw_dict[code].data];
                }
            }
            code = lzw_dict[code].prev;
        }

        image_index += match_len;
    }
}

// array size is 207703
const unsigned char gif[]  = {};