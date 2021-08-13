#if 1	// <-

#define htons(x) ((int16_t) ((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF)))
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#define DNS_IP "8.8.8.8"
#define DNS_PORT "53"

#define QUERY_FLAG               (0)
#define OPCODE_STANDARD_QUERY    (0)
#define RECURSION_DESIRED_FLAG   (1<<8)
#define LABEL_COMPRESSION_MASK   (0xC0)

#define TYPE_A                   (0x0001)
#define CLASS_IN                 (0x0001)

    //                                    1  1  1  1  1  1
    //      0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                      ID                       |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |QR|   Opcode  |AA|TC|RD|RA|   Z    |   RCODE   |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    QDCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ANCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    NSCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ARCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+

typedef struct
{
 int16_t id;
 int16_t flags;
 int16_t qdcount;
 int16_t ancount;
 int16_t nscount;
 int16_t arcount;
} T_DNS, *P_DNS;

typedef struct
{
 int16_t type;
 int16_t dnsClass;
 int32_t ttl;
 int16_t len;
} T_DNSHDR, *P_DNSHDR;

#define IP_ADDRESS_LEN 16	/* ip address buffer length */

char *htonsCpy(char *p, int16_t val);
int dnsMsg(char *buffer, int buflen, const char *name);
char *dnsDecode(char *buffer, int len, char *ip);
char dnsLookup(char *buf, const char *hostName);

#endif // ->
