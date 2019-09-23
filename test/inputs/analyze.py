import sys

class DumpParser:
  def __init__(self, filename):
    self.filename = filename
    self.f = open(filename, 'r')
    self.expect_timestep = False
  def readline(self):
    line = self.f.readline()
    if self.expect_timestep:
      self.timestep = line.rstrip()
      self.expect_timestep = False
    if line.startswith('ITEM: TIMESTEP'):
      self.expect_timestep = True
    return line

a, b = sys.argv[1:]
a, b = DumpParser(a), DumpParser(b)

def red(s):
  return '\033[31m{}\033[0m'.format(s)
def blue(s):
  return '\033[34m{}\033[0m'.format(s)
def green(s):
  return '\033[92m{}\033[0m'.format(s)

success = False
while True:
  aa, bb = a.readline(), b.readline()
  if not aa and not bb:
    success = True
    break
  if not aa or not bb: break
  if aa == bb: continue
  assert a.timestep == b.timestep
  aa, bb = aa.rstrip(), bb.rstrip()
  aparts = aa.split()
  bparts = bb.split()
  assert aparts[:2] == bparts[:2] # Id, type and positions are same
  if all([abs(float(aparts[5+i])-float(bparts[5+i])) < 1e-5 for i in range(3)]): continue
  cparts = []
  for i in range(3):
    ta = aparts[5+i]
    tb = bparts[5+i]
    if ta == tb: continue
    da, ea = ta.split('e')
    db, eb = tb.split('e')
    if ea != eb:
      cparts.append(red(ta) + blue(tb))
      continue
    dpos = 0
    for i in range(min(len(da), len(db))):
      dpos = i
      if da[i] != db[i]:
        break
    assert ta[:dpos] == tb[:dpos]
    if len(da[dpos:]) == 1 and len(db[dpos:]) == 1: continue
    cparts.append(ta[:dpos] + red(ta[dpos:]) + blue(tb[dpos:]))

  if not cparts: continue
  print('t= {:>3} a= {:>6} : {}'.format(a.timestep, aparts[0], ' '.join(cparts)))
if success:
  print(green('Yay!'))
